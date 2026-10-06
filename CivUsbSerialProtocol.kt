/* GPL-2.0-or-later. CI-V USB serial setup, based on Silicon Labs AN571. */
package com.isaklab.libcivk

import java.io.IOException
import java.nio.ByteBuffer
import java.nio.ByteOrder

/** USB control transfers are isolated so setup failures and modem-line safety are JVM-testable. */
internal object CivUsbSerialProtocol {
    fun isCp210x(vendor: Int, product: Int): Boolean = vendor == 0x10C4 && product in setOf(
        0xEA60, 0xEA61, 0xEA63, 0xEA70, 0xEA7A,
    )

    fun probeAddresses(configured: Int): IntArray = if (configured != 0) intArrayOf(configured) else intArrayOf(
        CivModels.ADDR_IC7300, CivModels.ADDR_IC7610, CivModels.ADDR_IC705,
        CivModels.ADDR_IC9700, CivModels.ADDR_IC905, CivModels.ADDR_ICR8600, CivModels.ADDR_IC7851,
    )

    /** Prove which UART is CI-V using only an address query, never by asserting SEND lines. */
    fun probeCiv(addresses: IntArray, write: (ByteArray) -> Unit, read: (ByteArray) -> Int,
                 nowNanos: () -> Long = System::nanoTime): Boolean {
        val deframer = CivProtocol.Deframer()
        val buffer = ByteArray(4096)
        for (address in addresses) {
            write(CivProtocol.readTransceiverId(address))
            val deadline = nowNanos() + 300_000_000L
            while (nowNanos() < deadline) {
                val count = read(buffer)
                if (count <= 0) continue
                for (body in deframer.push(buffer, count)) {
                    val frame = CivProtocol.parseFrame(body) as? CivProtocol.Frame.Message ?: continue
                    if (frame.from == address && frame.to == CivProtocol.CONTROLLER_ADDR &&
                        frame.cmd == CivProtocol.CMD_READ_ID && frame.data.size == 2 &&
                        frame.data[0].toInt() == CivProtocol.SUB_ID) return true
                }
            }
        }
        return false
    }

    /** AN571 §§5.1, 5.5–5.9, 5.11: 8N1, no flow control, DTR/RTS inactive. */
    fun configureCp210x(baud: Int, index: Int, transfer: (Int, Int, Int, Int, ByteArray?) -> Int) {
        require(baud > 0)
        fun out(request: Int, value: Int = 0, data: ByteArray? = null) {
            if (transfer(0x41, request, value, index, data) != (data?.size ?: 0))
                throw IOException("CP210x setup request 0x${request.toString(16)} failed")
        }
        fun read(request: Int, count: Int): ByteArray = ByteArray(count).also {
            if (transfer(0xC1, request, 0, index, it) != count) throw IOException("CP210x setup read-back failed")
        }
        out(0x00, 1) // IFC_ENABLE
        try {
            // Never assert serial modem lines: radios can assign them to SEND/CW.
            out(0x13, data = ByteArray(16)) // SET_FLOW: no hardware/software flow, inactive outputs
            out(0x07, 0x0300) // SET_MHS: change DTR/RTS, both inactive
            out(0x1E, data = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(baud).array())
            out(0x03, 0x0800) // SET_LINE_CTL: 8N1
            val actualBaud = ByteBuffer.wrap(read(0x1D, 4)).order(ByteOrder.LITTLE_ENDIAN).int
            val line = ByteBuffer.wrap(read(0x04, 2)).order(ByteOrder.LITTLE_ENDIAN).short.toInt() and 0xFFFF
            if (actualBaud != baud || line != 0x0800 || read(0x08, 1)[0].toInt() and 3 != 0)
                throw IOException("CP210x serial settings were not confirmed")
        } catch (e: Exception) {
            transfer(0x41, 0x00, 0, index, null)
            throw e
        }
    }

    fun configureCdc(baud: Int, index: Int, transfer: (Int, Int, Int, Int, ByteArray?) -> Int) {
        require(baud > 0)
        val coding = ByteBuffer.allocate(7).order(ByteOrder.LITTLE_ENDIAN).putInt(baud).put(0).put(0).put(8).array()
        if (transfer(0x21, 0x20, 0, index, coding) != coding.size ||
            transfer(0x21, 0x22, 0, index, null) != 0) throw IOException("CDC serial setup failed")
        val actual = ByteArray(7)
        if (transfer(0xA1, 0x21, 0, index, actual) != actual.size || !actual.contentEquals(coding))
            throw IOException("CDC serial settings were not confirmed")
    }
}
