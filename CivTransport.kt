/*
 * libcivk - Icom CI-V CAT driver for the iSDR driver host
 *
 * Copyright (C) 2026 Isak Ruas <isakruas@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */
package com.isaklab.libcivk

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbConstants
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbDeviceConnection
import android.hardware.usb.UsbEndpoint
import android.hardware.usb.UsbInterface
import android.hardware.usb.UsbManager
import android.util.Log
import androidx.core.content.ContextCompat
import java.io.IOException
import java.net.Socket
import java.net.SocketTimeoutException
import kotlin.coroutines.resume
import kotlinx.coroutines.suspendCancellableCoroutine

/**
 * Byte transport under the CI-V codec.
 *
 * The seam exists so the client can be exercised against a scripted fake;
 * the real implementations are the rig's USB-CDC serial interface and a TCP
 * bridge (ser2net and friends).
 */
interface CivTransport {
    /** Write every byte of [bytes]; throws on a dead link. */
    @Throws(IOException::class)
    fun writeAll(bytes: ByteArray)

    /**
     * Blocking read with the transport's own timeout (~50 ms — short enough
     * that disconnect is prompt, long enough not to spin); returns 0 on
     * timeout and throws on a dead link.
     */
    @Throws(IOException::class)
    fun readSome(buf: ByteArray): Int

    /** Release the port. Idempotent. */
    fun close() {}

    companion object {
        /** Read-poll timeout shared by the real transports, in ms. */
        const val READ_TIMEOUT_MS = 50
    }
}

/** CI-V over a TCP serial bridge. The constructor connects, and throws when it cannot. */
class TcpTransport(host: String, port: Int) : CivTransport {
    private val socket = Socket(host, port).apply {
        soTimeout = CivTransport.READ_TIMEOUT_MS
        tcpNoDelay = true
    }
    private val input = socket.getInputStream()
    private val output = socket.getOutputStream()

    @Throws(IOException::class)
    override fun writeAll(bytes: ByteArray) {
        output.write(bytes)
        output.flush()
    }

    @Throws(IOException::class)
    override fun readSome(buf: ByteArray): Int = try {
        val n = input.read(buf)
        if (n < 0) throw IOException("connection closed") else n
    } catch (_: SocketTimeoutException) {
        0
    }

    override fun close() {
        try {
            socket.close()
        } catch (_: Exception) {
        }
    }
}

/**
 * CI-V over Silicon Labs CP210x or CDC-ACM on the Android USB host bus.
 *
 * [open] must complete before the transport is handed to the client: it
 * enumerates a supported serial interface, walks the Android
 * USB permission dialog, claims both the communication and data interfaces,
 * and verifies the line coding ([baud] 8N1). DTR/RTS remain inactive because
 * radios can assign those lines to SEND/CW.
 */
class UsbCdcTransport(
    private val context: Context,
    private val baud: Int,
) : CivTransport {
    companion object {
        private const val TAG = "UsbCdcTransport"

        /** CDC class request: program baud rate, stop bits, parity, data bits. */
        private const val SET_LINE_CODING = 0x20

        /** CDC class request: assert/deassert DTR (bit 0) and RTS (bit 1). */
        private const val SET_CONTROL_LINE_STATE = 0x22

        /** Host-to-device, class, interface-recipient control transfer. */
        private const val REQTYPE_CLASS_INTERFACE_OUT = 0x21

        private const val CONTROL_TIMEOUT_MS = 500
        private const val WRITE_TIMEOUT_MS = 500
    }

    private var connection: UsbDeviceConnection? = null
    private var cp210xInterface: Int? = null
    private var commInterface: UsbInterface? = null
    private var dataInterface: UsbInterface? = null
    private var endpointIn: UsbEndpoint? = null
    private var endpointOut: UsbEndpoint? = null
    private var detachReceiver: BroadcastReceiver? = null

    @Volatile private var deviceLost = false

    private val actionUsbPermission = "${context.packageName}.USB_PERMISSION_CIV"

    /**
     * Find, get permission for, and configure a CDC serial device. False
     * when no device is present, permission was denied, or the claim failed.
     */
    suspend fun open(probeCivAddress: Int? = null): Boolean {
        val usbManager = context.getSystemService(Context.USB_SERVICE) as UsbManager
        val devices = usbManager.deviceList.values.filter {
            hasCdcDataInterface(it) || CivUsbSerialProtocol.isCp210x(it.vendorId, it.productId)
        }
        if (devices.isEmpty()) {
            Log.w(TAG, "no supported USB serial device attached")
            return false
        }
        for (device in devices) {
            if (!usbManager.hasPermission(device) && !requestPermission(usbManager, device)) {
                Log.w(TAG, "USB permission denied for ${device.deviceName}")
                continue
            }
            val conn = usbManager.openDevice(device) ?: continue

            val cp210x = CivUsbSerialProtocol.isCp210x(device.vendorId, device.productId)
            val interfaces = (0 until device.interfaceCount).map(device::getInterface)
            val comm = if (cp210x) null else interfaces.firstOrNull { it.interfaceClass == UsbConstants.USB_CLASS_COMM }
            val candidates = interfaces.filter {
                it.interfaceClass == if (cp210x) UsbConstants.USB_CLASS_VENDOR_SPEC else UsbConstants.USB_CLASS_CDC_DATA
            }.sortedBy { it.id }
            if (comm != null && !conn.claimInterface(comm, true)) { conn.close(); continue }
            for (data in candidates) {
                val endpoints = (0 until data.endpointCount).map(data::getEndpoint)
                    .filter { it.type == UsbConstants.USB_ENDPOINT_XFER_BULK }
                val epIn = endpoints.firstOrNull { it.direction == UsbConstants.USB_DIR_IN } ?: continue
                val epOut = endpoints.firstOrNull { it.direction == UsbConstants.USB_DIR_OUT } ?: continue
                if (!conn.claimInterface(data, true)) continue
                val controlIndex = if (cp210x) data.id else comm?.id ?: data.id
                try {
                    val transfer = { type: Int, request: Int, value: Int, index: Int, bytes: ByteArray? ->
                        conn.controlTransfer(type, request, value, index, bytes, bytes?.size ?: 0, CONTROL_TIMEOUT_MS)
                    }
                    if (cp210x) CivUsbSerialProtocol.configureCp210x(baud, controlIndex, transfer)
                    else CivUsbSerialProtocol.configureCdc(baud, controlIndex, transfer)
                    if (probeCivAddress != null && !CivUsbSerialProtocol.probeCiv(
                            CivUsbSerialProtocol.probeAddresses(probeCivAddress),
                            write = { bytes ->
                                var offset = 0
                                while (offset < bytes.size) {
                                    val remaining = bytes.copyOfRange(offset, bytes.size)
                                    val count = conn.bulkTransfer(epOut, remaining, remaining.size, WRITE_TIMEOUT_MS)
                                    if (count <= 0) throw IOException("CI-V USB probe write failed")
                                    offset += count
                                }
                            },
                            read = { bytes -> conn.bulkTransfer(epIn, bytes, bytes.size, CivTransport.READ_TIMEOUT_MS) },
                        )) throw IOException("no correlated CI-V address reply on UART ${data.id}")

                    cp210xInterface = if (cp210x) controlIndex else null
                    connection = conn
                    commInterface = comm
                    dataInterface = data
                    endpointIn = epIn
                    endpointOut = epOut
                    registerDetachReceiver(device)
                    Log.i(TAG, "opened ${device.deviceName} interface ${data.id} at $baud baud")
                    return true
                } catch (e: Exception) {
                    Log.w(TAG, "USB serial interface ${data.id} rejected: ${e.message}")
                    if (cp210x) conn.controlTransfer(0x41, 0x00, 0, controlIndex, null, 0, CONTROL_TIMEOUT_MS)
                    conn.releaseInterface(data)
                }
            }
            comm?.let { conn.releaseInterface(it) }
            conn.close()
        }
        return false
    }

    private fun hasCdcDataInterface(device: UsbDevice): Boolean {
        for (i in 0 until device.interfaceCount) {
            val iface = device.getInterface(i)
            if (iface.interfaceClass != UsbConstants.USB_CLASS_CDC_DATA) continue
            var hasIn = false
            var hasOut = false
            for (e in 0 until iface.endpointCount) {
                val ep = iface.getEndpoint(e)
                if (ep.type != UsbConstants.USB_ENDPOINT_XFER_BULK) continue
                if (ep.direction == UsbConstants.USB_DIR_IN) hasIn = true else hasOut = true
            }
            if (hasIn && hasOut) return true
        }
        return false
    }

    @Throws(IOException::class)
    override fun writeAll(bytes: ByteArray) {
        val conn = connection ?: throw IOException("port not open")
        val ep = endpointOut ?: throw IOException("port not open")
        if (deviceLost) throw IOException("device lost")
        var off = 0
        while (off < bytes.size) {
            val chunk = if (off == 0) bytes else bytes.copyOfRange(off, bytes.size)
            val n = conn.bulkTransfer(ep, chunk, chunk.size, WRITE_TIMEOUT_MS)
            if (n <= 0) throw IOException("bulk write made no progress")
            off += n
        }
    }

    @Throws(IOException::class)
    override fun readSome(buf: ByteArray): Int {
        val conn = connection ?: throw IOException("port not open")
        val ep = endpointIn ?: throw IOException("port not open")
        if (deviceLost) throw IOException("device lost")
        // bulkTransfer reports timeout and failure identically (negative);
        // the detach receiver is what turns a pulled cable into an error
        // instead of an endless quiet timeout.
        val n = conn.bulkTransfer(ep, buf, buf.size, CivTransport.READ_TIMEOUT_MS)
        return if (n > 0) n else 0
    }

    override fun close() {
        unregisterDetachReceiver()
        val conn = connection ?: return
        connection = null
        try {
            cp210xInterface?.let { conn.controlTransfer(0x41, 0x00, 0, it, null, 0, CONTROL_TIMEOUT_MS) }
            cp210xInterface = null
            dataInterface?.let { conn.releaseInterface(it) }
            commInterface?.let { conn.releaseInterface(it) }
        } catch (_: Exception) {
        }
        try {
            conn.close()
        } catch (_: Exception) {
        }
    }

    // ---- USB permission -----------------------------------------------------

    private suspend fun requestPermission(usbManager: UsbManager, device: UsbDevice): Boolean =
        suspendCancellableCoroutine { continuation ->
            val receiver = object : BroadcastReceiver() {
                override fun onReceive(ctx: Context, intent: Intent) {
                    if (intent.action != actionUsbPermission) return
                    @Suppress("DEPRECATION")
                    val grantedDevice: UsbDevice? =
                        intent.getParcelableExtra(UsbManager.EXTRA_DEVICE)
                    if (grantedDevice != null && grantedDevice.deviceName != device.deviceName) {
                        return // grant for a different device
                    }
                    try {
                        context.unregisterReceiver(this)
                    } catch (_: Exception) {
                    }
                    val granted =
                        intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)
                    if (continuation.isActive) continuation.resume(granted)
                }
            }
            ContextCompat.registerReceiver(
                context, receiver, IntentFilter(actionUsbPermission),
                ContextCompat.RECEIVER_NOT_EXPORTED,
            )

            // The system fills in extras, so the PendingIntent must be mutable
            // and the intent explicit (package-scoped) on modern Android.
            val flags = if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.S) {
                PendingIntent.FLAG_MUTABLE
            } else {
                0
            }
            val pendingIntent = PendingIntent.getBroadcast(
                context, 0,
                Intent(actionUsbPermission).setPackage(context.packageName),
                flags,
            )
            usbManager.requestPermission(device, pendingIntent)

            continuation.invokeOnCancellation {
                try {
                    context.unregisterReceiver(receiver)
                } catch (_: Exception) {
                }
            }
        }

    // ---- hot-unplug detection -------------------------------------------------

    /**
     * A pulled cable never errors a pending bulkTransfer timeout, so without
     * this the reader would poll a dead port forever. The system detach
     * broadcast fires the moment the kernel drops the device; the next
     * read/write then throws and the client reports the link lost.
     */
    private fun registerDetachReceiver(device: UsbDevice) {
        val receiver = object : BroadcastReceiver() {
            override fun onReceive(ctx: Context, intent: Intent) {
                if (intent.action != UsbManager.ACTION_USB_DEVICE_DETACHED) return
                @Suppress("DEPRECATION")
                val detached: UsbDevice? = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE)
                if (detached == null || detached.deviceName != device.deviceName) return
                Log.e(TAG, "USB device detached, device lost")
                deviceLost = true
            }
        }
        detachReceiver = receiver
        // Protected system broadcast: only the OS can send it, so an exported
        // receiver is safe — and it must be exported to receive an implicit
        // system broadcast on modern Android.
        ContextCompat.registerReceiver(
            context, receiver, IntentFilter(UsbManager.ACTION_USB_DEVICE_DETACHED),
            ContextCompat.RECEIVER_EXPORTED,
        )
    }

    private fun unregisterDetachReceiver() {
        detachReceiver?.let {
            try {
                context.unregisterReceiver(it)
            } catch (_: Exception) {
                // already unregistered
            }
            detachReceiver = null
        }
    }
}
