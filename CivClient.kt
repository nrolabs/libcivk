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

import com.isaklab.isdrdrivers.core.CatControlCapable
import com.isaklab.isdrdrivers.core.CatRepeaterCapable
import com.isaklab.isdrdrivers.core.RadioClient
import com.isaklab.isdrdrivers.core.TransmitCapable
import com.isaklab.isdrproto.CatRepeater
import com.isaklab.isdrproto.CatRepeaterConfig
import com.isaklab.libcivk.CivProtocol as P
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.CountDownLatch
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicLong
import kotlin.concurrent.thread
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

/**
 * CI-V rig client: request/response over the shared bus, unsolicited
 * transceive tracking, and scope-waveform delivery on the spectrum plane.
 *
 * One reader thread owns the transport. Control calls enqueue their frame
 * and wait for the matching reply; the reader also folds in everything the
 * rig volunteers (frequency/mode transceive, scope sweeps). There is no IQ
 * plane: every data delivery is `onDataReceived(spectrumDb, empty)`.
 */
class CivClient(
    transport: CivTransport,
    /** The rig's CI-V bus address, or 0 to probe for one. */
    configuredAddr: Int,
    /** (power spectrum in dB, interleaved IQ — always empty for CI-V) */
    private val onDataReceived: (FloatArray, FloatArray) -> Unit,
    private val onConnectionStatusChanged: (Boolean, String) -> Unit,
) : RadioClient, TransmitCapable, CatControlCapable, CatRepeaterCapable {

    companion object {
        /** How long one request waits for its reply before retrying, in ms. */
        private const val REPLY_TIMEOUT_MS = 300L

        /** Attempts per request; the bus is multi-drop and collisions are normal. */
        private const val ATTEMPTS = 3

        /** Cancellation is observed without cutting a frame write in half. */
        private const val CANCEL_POLL_MS = 5L

        /** Quarantine for duplicate untagged ACK/NAK frames after a retry. */
        private const val RETRY_DRAIN_MS = 600L

        private const val REPEATER_CANCELLED =
            "CI-V repeater transaction cancelled for priority unkey"

        private val EMPTY_FLOATS = FloatArray(0)

        /** Bus addresses tried, in order, when the configured address is 0. */
        private val PROBE_ADDRS = intArrayOf(
            CivModels.ADDR_IC7300,
            CivModels.ADDR_IC7610,
            CivModels.ADDR_IC705,
            CivModels.ADDR_IC9700,
            CivModels.ADDR_IC905,
            CivModels.ADDR_ICR8600,
            CivModels.ADDR_IC7851,
            0x88, 0x8C, 0xA0, 0x70, 0x5E,
        )
    }

    private class Pending(val cmd: Int, val rigAddr: Int) {
        /** null data = NAK / rejected. */
        val reply = ArrayBlockingQueue<Result<ByteArray>>(1)
    }

    private class OutgoingFrame(val bytes: ByteArray) {
        /** Signals only after the complete frame has left writeAll. */
        val written = CountDownLatch(1)
    }

    private val configuredAddr = configuredAddr and 0xFF
    private var transport: CivTransport? = transport

    private val freqHz = AtomicLong(0)
    private val spanHz = AtomicLong(0)
    private val lowEdgeHz = AtomicLong(0)
    private val highEdgeHz = AtomicLong(0)
    private val modeCode = AtomicInteger(-1)
    private val ptt = AtomicBoolean(false)
    private val running = AtomicBoolean(false)
    private val rigAddrAtomic = AtomicInteger(this.configuredAddr)

    private val outgoing = LinkedBlockingQueue<OutgoingFrame>()
    private val pendingLock = Object()
    private val busLock = Object()
    private val readerCycle = AtomicLong(0)
    private var pending: Pending? = null

    @Volatile private var activeRepeater: CatRepeaterConfig? = null
    @Volatile private var repeaterUncertain = false
    @Volatile private var repeaterMutationStarted = false
    private val repeaterLifecycleLock = Object()
    private val repeaterInFlight = AtomicBoolean(false)
    private val repeaterCancel = AtomicBoolean(false)

    private var reader: Thread? = null
    private var started = false
    private var scopeCaps: CivModels.ScopeCaps? = null
    @Volatile private var stateListener: (() -> Unit)? = null

    @Volatile private var spectrumWanted = true

    override var spectrumEnabled: Boolean
        get() = spectrumWanted
        set(value) {
            spectrumWanted = value
            // ACK/NAK has no command tag, so even this UI control uses the
            // same serialized transaction boundary as RF-affecting writes.
            if (scopeCaps != null) {
                transact(P.scopeWaveOutput(rigAddr(), value), P.CMD_SCOPE)
            }
        }

    override fun setStateListener(listener: (() -> Unit)?) {
        stateListener = listener
    }

    private fun rigAddr(): Int = rigAddrAtomic.get()

    fun modelName(): String {
        val addr = rigAddr()
        return CivModels.modelName(addr) ?: "CI-V 0x%02X".format(addr)
    }

    fun scopeCapable(): Boolean = scopeCaps != null

    fun mode(): Int = modeCode.get()

    /** The edges of the last delivered sweep, in Hz. */
    fun scopeEdges(): Pair<Long, Long> = Pair(lowEdgeHz.get(), highEdgeHz.get())

    // ---- request/response -------------------------------------------------------

    /**
     * Send [frame] and wait for the rig's reply to command [cmd], retrying
     * through collisions and timeouts. Null on NAK, timeout or teardown.
     */
    private fun transact(frame: ByteArray, cmd: Int): ByteArray? =
        transactResult(frame, cmd).getOrNull()

    /**
     * One request/reply on the shared bus. A single pending receiver spans
     * all retries; afterwards untagged ACK/NAK duplicates are quarantined
     * before another caller may install a receiver.
     */
    private fun transactResult(
        frame: ByteArray,
        cmd: Int,
        cancellable: Boolean = false,
    ): Result<ByteArray> {
        if (!running.get()) return Result.failure(IllegalStateException("not connected"))
        return synchronized(busLock) {
            if (cancellable && repeaterCancel.get()) {
                return@synchronized Result.failure(IllegalStateException(REPEATER_CANCELLED))
            }
            val target = frame.getOrNull(2)?.toInt()?.and(0xFF)
                ?: return@synchronized Result.failure(
                    IllegalArgumentException("malformed outgoing CI-V frame"),
                )
            val p = Pending(cmd, target)
            synchronized(pendingLock) { pending = p }
            var attempts = 0
            var sent = 0
            var receivedReply = false
            var cancelled = false
            var result: Result<ByteArray>? = null
            var lastError = "no reply"

            while (attempts < ATTEMPTS && result == null) {
                if (cancellable && repeaterCancel.get()) {
                    cancelled = true
                    result = Result.failure(IllegalStateException(REPEATER_CANCELLED))
                    break
                }
                val outgoingFrame = OutgoingFrame(frame)
                outgoing.offer(outgoingFrame)
                sent++
                val deadline = System.nanoTime() +
                    TimeUnit.MILLISECONDS.toNanos(REPLY_TIMEOUT_MS)
                while (result == null) {
                    if (cancellable && repeaterCancel.get()) {
                        // Do not let priority unkey overtake bytes still being written.
                        while (running.get() &&
                            !outgoingFrame.written.await(CANCEL_POLL_MS, TimeUnit.MILLISECONDS)
                        ) {
                            // Wait only at the proven complete-frame boundary.
                        }
                        cancelled = true
                        result = Result.failure(IllegalStateException(REPEATER_CANCELLED))
                        break
                    }
                    val remaining = deadline - System.nanoTime()
                    if (remaining <= 0) {
                        lastError = "timeout"
                        break
                    }
                    val slice = if (cancellable) {
                        minOf(remaining, TimeUnit.MILLISECONDS.toNanos(CANCEL_POLL_MS))
                    } else {
                        remaining
                    }
                    val reply = p.reply.poll(slice, TimeUnit.NANOSECONDS)
                    if (reply != null) {
                        receivedReply = true
                        result = reply
                    }
                }
                attempts++
            }
            if (result == null) {
                result = Result.failure(IllegalStateException(lastError))
            }
            synchronized(pendingLock) { if (pending === p) pending = null }

            val timeout = result.exceptionOrNull()?.message == "timeout"
            if (sent > 1 && receivedReply || sent > 0 && (timeout || cancelled)) {
                val drainMs = if (cancelled && sent == 1) {
                    REPLY_TIMEOUT_MS
                } else {
                    RETRY_DRAIN_MS
                }
                try {
                    Thread.sleep(drainMs)
                } catch (_: InterruptedException) {
                    Thread.currentThread().interrupt()
                }
                // Require a reader pass after the drain, so an ACK already
                // buffered by the transport cannot satisfy the next command.
                val cycle = readerCycle.get()
                val boundaryDeadline = System.nanoTime() +
                    TimeUnit.MILLISECONDS.toNanos(REPLY_TIMEOUT_MS)
                while (running.get() && readerCycle.get() == cycle &&
                    System.nanoTime() < boundaryDeadline
                ) {
                    try {
                        Thread.sleep(1)
                    } catch (_: InterruptedException) {
                        Thread.currentThread().interrupt()
                        break
                    }
                }
                if (running.get() && readerCycle.get() == cycle) {
                    running.set(false)
                    result = Result.failure(
                        IllegalStateException(
                            "CI-V reply quarantine could not establish a safe bus boundary; " +
                                "link stopped",
                        ),
                    )
                }
            }
            result
        }
    }

    // ---- rig control --------------------------------------------------------

    /** The rig IF filter (FIL1..3) the last mode write carried. */
    private var lastFil = 1

    /**
     * Set the operating mode (CI-V mode code), carrying the rig IF filter
     * selection the last CATCTL_FIL chose (FIL1 on a fresh session).
     */
    fun setMode(mode: Int): Boolean {
        if (repeaterInFlight.get()) return false
        val frame = P.writeMode(rigAddr(), mode, lastFil) ?: return false
        val ok = transact(frame, P.CMD_WRITE_MODE) != null
        if (ok) modeCode.set(mode)
        return ok
    }

    /**
     * One of the rig's own receive controls (CATCTL_* ids). Returns false
     * for an id this dialect has no wire for — the caller treats that as
     * "the rig does not have it", never as an error.
     */
    fun setControl(id: Int, value: Int): Boolean {
        if (repeaterInFlight.get()) return false
        val addr = rigAddr()
        return when (id) {
            // CATCTL_FIL: the mode write (0x06) carries the IF filter byte.
            1 -> {
                if (value !in 1..3) return false
                lastFil = value
                val mode = modeCode.get()
                if (mode >= 0) setMode(mode) else true
            }
            // CATCTL_RF_GAIN / CATCTL_SQUELCH / CATCTL_PBT_IN /
            // CATCTL_PBT_OUT / CATCTL_AF_GAIN: plain 0..255 levels.
            2, 3, 10, 11, 13 -> {
                if (value !in 0..255) return false
                val sub = when (id) {
                    2 -> P.SUB_LEVEL_RF
                    3 -> P.SUB_LEVEL_SQL
                    10 -> P.SUB_LEVEL_PBT_IN
                    11 -> P.SUB_LEVEL_PBT_OUT
                    else -> P.SUB_LEVEL_AF
                }
                transact(P.setLevel(addr, sub, value), P.CMD_LEVEL) != null
            }
            // CATCTL_NR: 0 = off; 1..15 = on, then the strength on the
            // rig's 0..255 level scale (15 steps of 17).
            4 -> {
                if (value !in 0..15) return false
                val on = value > 0
                if (transact(
                        P.setFunc(addr, P.SUB_FUNC_NR, if (on) 1 else 0), P.CMD_FUNC,
                    ) == null
                ) {
                    return false
                }
                if (!on) return true
                val level = value * 255 / 15
                transact(P.setLevel(addr, P.SUB_LEVEL_NR, level), P.CMD_LEVEL) != null
            }
            // CATCTL_NB / CATCTL_NOTCH_AUTO: on/off functions.
            5, 6 -> {
                if (value !in 0..1) return false
                val sub = if (id == 5) P.SUB_FUNC_NB else P.SUB_FUNC_NOTCH_AUTO
                transact(P.setFunc(addr, sub, value), P.CMD_FUNC) != null
            }
            // CATCTL_AGC: 1 fast, 2 mid, 3 slow. These rigs have no AGC-off
            // code on this command, so 0 is unsupported here.
            7 -> {
                if (value !in 1..3) return false
                transact(P.setFunc(addr, P.SUB_FUNC_AGC, value), P.CMD_FUNC) != null
            }
            // CATCTL_PREAMP: 0 off, 1/2 = stage.
            8 -> {
                if (value !in 0..2) return false
                transact(P.setFunc(addr, P.SUB_FUNC_PREAMP, value), P.CMD_FUNC) != null
            }
            // CATCTL_ATT: dB as one BCD byte, 0 = off. Accept anything the
            // wire can carry up to 45 dB; the rig NAKs a step it lacks.
            9 -> {
                if (value !in 0..45) return false
                val frame = P.setAttenuator(addr, value) ?: return false
                transact(frame, P.CMD_ATTENUATOR) != null
            }
            // CATCTL_FILTER_WIDTH: 0x1A 0x03 with the per-mode width code.
            // 0x1A subcommands are model-family specific — only the known
            // scope-capable family is proven to put the width there, so an
            // unknown address gets false rather than a wrong setting.
            12 -> {
                if (scopeCaps == null) return false
                val mode = modeCode.get()
                if (mode < 0) return false
                val frame = P.setFilterWidth(addr, mode, value) ?: return false
                transact(frame, P.CMD_MEM) != null
            }
            // CATCTL_RF_POWER: setting an RF-affecting value is successful
            // only after an independent read returns the exact 0..255 level.
            14 -> {
                if (value !in 0..255 || !CivModels.rigCaps(addr).hasTx) return false
                if (transact(P.setLevel(addr, P.SUB_LEVEL_RFPOWER, value), P.CMD_LEVEL) == null) {
                    return false
                }
                val actual = transact(P.readLevel(addr, P.SUB_LEVEL_RFPOWER), P.CMD_LEVEL)
                    ?.let { P.parseLevel(it, P.SUB_LEVEL_RFPOWER) }
                actual == value
            }
            else -> false
        }
    }

    override fun setCatMode(mode: Int): Boolean = setMode(mode)

    override fun currentCatMode(): Int = mode()

    override fun setCatControl(id: Int, value: Int): Boolean = setControl(id, value)

    override fun catRepeaterCapabilities(): Int = CivModels.rigCaps(rigAddr()).repeaterCaps

    override fun requestCatRepeaterCancelForUnkey() {
        synchronized(repeaterLifecycleLock) {
            if (repeaterInFlight.get()) repeaterCancel.set(true)
        }
    }

    override fun setCatRepeater(config: CatRepeaterConfig): String? {
        synchronized(repeaterLifecycleLock) {
            if (!repeaterInFlight.compareAndSet(false, true)) {
                return "another CI-V repeater transaction is already in progress"
            }
            repeaterCancel.set(false)
            repeaterMutationStarted = false
        }
        return try {
            setCatRepeaterActive(config)
        } catch (error: Exception) {
            if (repeaterMutationStarted) repeaterUncertain = true
            "CI-V repeater transaction failed: ${error.message ?: error.javaClass.simpleName}"
        } finally {
            synchronized(repeaterLifecycleLock) {
                repeaterMutationStarted = false
                repeaterInFlight.set(false)
                repeaterCancel.set(false)
            }
        }
    }

    // ---- exact repeater transaction ---------------------------------------

    private fun repeaterProfile(): CivRepeaterProfile =
        civRepeaterProfile(rigAddr(), freqHz.get())

    private fun wireBytes(data: ByteArray): String = data.joinToString(
        prefix = "[",
        postfix = "]",
    ) { "0x%02X".format(it.toInt() and 0xFF) }

    private fun transactNamed(
        frame: ByteArray,
        cmd: Int,
        action: String,
        cancellable: Boolean,
    ): Result<ByteArray> = transactResult(frame, cmd, cancellable).fold(
        onSuccess = { Result.success(it) },
        onFailure = {
            Result.failure(IllegalStateException("$action failed: ${it.message ?: "unknown error"}"))
        },
    )

    private fun writeCommand(
        frame: ByteArray,
        cmd: Int,
        action: String,
        cancellable: Boolean,
    ): String? = transactNamed(frame, cmd, action, cancellable).exceptionOrNull()?.message

    private fun readFuncValue(sub: Int, cancellable: Boolean): Result<Int> {
        val data = transactNamed(
            P.readFunc(rigAddr(), sub),
            P.CMD_FUNC,
            "reading CI-V function 0x%02X".format(sub),
            cancellable,
        ).getOrElse { return Result.failure(it) }
        return if (data.size == 2 && (data[0].toInt() and 0xFF) == sub) {
            Result.success(data[1].toInt() and 0xFF)
        } else {
            Result.failure(
                IllegalStateException(
                    "malformed CI-V function 0x%02X read-back %s".format(
                        sub,
                        wireBytes(data),
                    ),
                ),
            )
        }
    }

    private fun setFuncValue(sub: Int, value: Int, cancellable: Boolean): String? =
        writeCommand(
            P.setFunc(rigAddr(), sub, value),
            P.CMD_FUNC,
            "setting CI-V function 0x%02X".format(sub),
            cancellable,
        )

    private fun readCtcss(sub: Int, cancellable: Boolean): Result<Int> {
        val frame = P.readTone(rigAddr(), sub)
            ?: return Result.failure(IllegalArgumentException("invalid CI-V CTCSS sub-command"))
        val data = transactNamed(
            frame,
            P.CMD_TONE,
            "reading CI-V CTCSS 0x%02X".format(sub),
            cancellable,
        ).getOrElse { return Result.failure(it) }
        return P.parseCtcssTone(data, sub)?.let { Result.success(it) }
            ?: Result.failure(
                IllegalStateException(
                    "malformed CI-V CTCSS 0x%02X read-back %s".format(
                        sub,
                        wireBytes(data),
                    ),
                ),
            )
    }

    private fun writeCtcss(sub: Int, value: Int, cancellable: Boolean): String? {
        val frame = P.setCtcssTone(rigAddr(), sub, value)
            ?: return "cannot encode CI-V CTCSS value $value"
        return writeCommand(
            frame,
            P.CMD_TONE,
            "setting CI-V CTCSS 0x%02X".format(sub),
            cancellable,
        )
    }

    private fun readDcs(cancellable: Boolean): Result<Triple<Int, Int, Int>> {
        val frame = P.readTone(rigAddr(), P.SUB_TONE_DCS)
            ?: return Result.failure(IllegalStateException("cannot encode CI-V DCS read"))
        val data = transactNamed(
            frame,
            P.CMD_TONE,
            "reading CI-V DCS",
            cancellable,
        ).getOrElse { return Result.failure(it) }
        return P.parseDcs(data)?.let { Result.success(it) }
            ?: Result.failure(
                IllegalStateException("malformed CI-V DCS read-back ${wireBytes(data)}"),
            )
    }

    private fun readPhysicalPtt(cancellable: Boolean = false): Result<Boolean> {
        val data = transactNamed(
            P.readPtt(rigAddr()),
            P.CMD_PTT,
            "reading CI-V PTT state",
            cancellable,
        ).getOrElse { return Result.failure(it) }
        return when {
            data.size == 2 && (data[0].toInt() and 0xFF) == P.SUB_PTT && data[1].toInt() == 0 ->
                Result.success(false)
            data.size == 2 && (data[0].toInt() and 0xFF) == P.SUB_PTT && data[1].toInt() == 1 ->
                Result.success(true)
            else -> Result.failure(
                IllegalStateException("malformed CI-V PTT read-back ${wireBytes(data)}"),
            )
        }
    }

    private fun confirmPhysicalFm(cancellable: Boolean): String? {
        val data = transactNamed(
            P.readMode(rigAddr()),
            P.CMD_READ_MODE,
            "reading physical CI-V mode",
            cancellable,
        ).getOrElse { return it.message ?: "cannot read physical CI-V mode" }
        if (data.isEmpty()) return "malformed physical CI-V mode read-back []"
        val mode = data[0].toInt() and 0xFF
        modeCode.set(mode)
        return if (mode == P.MODE_FM) {
            null
        } else {
            "CI-V repeater controls require physical FM mode; read app-mode code $mode"
        }
    }

    private fun readRepeaterState(cancellable: Boolean): Result<CivRepeaterState> {
        val addr = rigAddr()
        val caps = CivModels.rigCaps(addr)
        val duplex = if (caps.repeaterCaps and CatRepeater.CAP_DUPLEX != 0) {
            val data = transactNamed(
                P.readDuplex(addr),
                P.CMD_DUPLEX,
                "reading CI-V duplex state",
                cancellable,
            ).getOrElse { return Result.failure(it) }
            P.parseDuplex(data) ?: return Result.failure(
                IllegalStateException("malformed CI-V duplex read-back ${wireBytes(data)}"),
            )
        } else {
            CatRepeater.DUPLEX_SIMPLEX
        }
        // Read even in simplex: rollback must preserve inactive operator presets.
        val offsetHz = if (caps.repeaterCaps and CatRepeater.CAP_OFFSET != 0) {
            val data = transactNamed(
                P.readRepeaterOffset(addr),
                P.CMD_READ_REPEATER_OFFSET,
                "reading CI-V repeater offset",
                cancellable,
            ).getOrElse { return Result.failure(it) }
            P.parseRepeaterOffset(data, caps.repeaterOffsetBytes) ?: return Result.failure(
                IllegalStateException(
                    "malformed CI-V repeater offset read-back ${wireBytes(data)}",
                ),
            )
        } else {
            0L
        }

        var toneMode: Int? = null
        var repeaterTone: Int? = null
        var toneSquelch: Int? = null
        var dcsEnabled: Int? = null
        when (addr) {
            CivModels.ADDR_IC705, CivModels.ADDR_IC9700, CivModels.ADDR_IC905 -> {
                toneMode = readFuncValue(P.SUB_FUNC_TONE_MODE, cancellable)
                    .getOrElse { return Result.failure(it) }
            }
            CivModels.ADDR_ICR8600 -> {
                toneSquelch = readFuncValue(P.SUB_FUNC_TONE_SQUELCH, cancellable)
                    .getOrElse { return Result.failure(it) }
                dcsEnabled = readFuncValue(P.SUB_FUNC_DCS, cancellable)
                    .getOrElse { return Result.failure(it) }
            }
            else -> {
                repeaterTone = readFuncValue(P.SUB_FUNC_REPEATER_TONE, cancellable)
                    .getOrElse { return Result.failure(it) }
                toneSquelch = readFuncValue(P.SUB_FUNC_TONE_SQUELCH, cancellable)
                    .getOrElse { return Result.failure(it) }
            }
        }
        val txCtcss = if (caps.repeaterCaps and CatRepeater.CAP_CTCSS_TX != 0) {
            readCtcss(P.SUB_TONE_TX, cancellable).getOrElse { return Result.failure(it) }
        } else {
            null
        }
        val rxCtcss = if (caps.repeaterCaps and CatRepeater.CAP_CTCSS_RX != 0) {
            readCtcss(P.SUB_TONE_RX, cancellable).getOrElse { return Result.failure(it) }
        } else {
            null
        }
        val dcs = if (caps.repeaterCaps and
            (CatRepeater.CAP_DCS_TX or CatRepeater.CAP_DCS_RX) != 0
        ) {
            readDcs(cancellable).getOrElse { return Result.failure(it) }
        } else {
            null
        }
        return Result.success(
            CivRepeaterState(
                duplex = duplex,
                offsetHz = offsetHz,
                toneMode = toneMode,
                repeaterToneEnabled = repeaterTone,
                toneSquelchEnabled = toneSquelch,
                dcsEnabled = dcsEnabled,
                txCtcss = txCtcss,
                rxCtcss = rxCtcss,
                dcs = dcs,
            ),
        )
    }

    private fun repeaterConfigFromState(state: CivRepeaterState): Result<CatRepeaterConfig> {
        val kinds = when (rigAddr()) {
            CivModels.ADDR_IC705, CivModels.ADDR_IC9700, CivModels.ADDR_IC905 -> {
                val mode = state.toneMode ?: return Result.failure(
                    IllegalStateException("missing CI-V cross-tone selector"),
                )
                civCrossToneKinds(mode) ?: return Result.failure(
                    IllegalStateException(
                        "unsupported CI-V cross-tone selector read-back 0x%02X".format(mode),
                    ),
                )
            }
            CivModels.ADDR_ICR8600 -> {
                state.dcs?.let {
                    if (it.second != CatRepeater.DCS_NORMAL) {
                        return Result.failure(
                            IllegalStateException(
                                "IC-R8600 reported non-zero fixed TX DCS polarity ${it.second}",
                            ),
                        )
                    }
                }
                val ctcss = state.toneSquelchEnabled ?: return Result.failure(
                    IllegalStateException("missing CI-V tone-squelch selector"),
                )
                val dcs = state.dcsEnabled ?: return Result.failure(
                    IllegalStateException("missing CI-V DCS selector"),
                )
                when (Pair(ctcss, dcs)) {
                    Pair(0, 0) -> Pair(CatRepeater.TONE_OFF, CatRepeater.TONE_OFF)
                    Pair(1, 0) -> Pair(CatRepeater.TONE_OFF, CatRepeater.TONE_CTCSS)
                    Pair(0, 1) -> Pair(CatRepeater.TONE_OFF, CatRepeater.TONE_DCS)
                    else -> return Result.failure(
                        IllegalStateException(
                            "ambiguous CI-V receiver tone flags: TSQL=$ctcss, DCS=$dcs",
                        ),
                    )
                }
            }
            else -> {
                val tone = state.repeaterToneEnabled ?: return Result.failure(
                    IllegalStateException("missing CI-V repeater-tone selector"),
                )
                val squelch = state.toneSquelchEnabled ?: return Result.failure(
                    IllegalStateException("missing CI-V tone-squelch selector"),
                )
                when {
                    tone == 0 && squelch == 0 ->
                        Pair(CatRepeater.TONE_OFF, CatRepeater.TONE_OFF)
                    tone == 1 && squelch == 0 ->
                        Pair(CatRepeater.TONE_CTCSS, CatRepeater.TONE_OFF)
                    tone in 0..1 && squelch == 1 ->
                        Pair(CatRepeater.TONE_CTCSS, CatRepeater.TONE_CTCSS)
                    else -> return Result.failure(
                        IllegalStateException(
                            "malformed CI-V legacy tone flags: TONE=$tone, TSQL=$squelch",
                        ),
                    )
                }
            }
        }

        var txValue = 0
        var txPolarity = CatRepeater.DCS_NORMAL
        var rxValue = 0
        var rxPolarity = CatRepeater.DCS_NORMAL
        if (kinds.first == CatRepeater.TONE_CTCSS) {
            txValue = state.txCtcss ?: return Result.failure(
                IllegalStateException("missing CI-V TX CTCSS read-back"),
            )
        }
        if (kinds.second == CatRepeater.TONE_CTCSS) {
            rxValue = state.rxCtcss ?: return Result.failure(
                IllegalStateException("missing CI-V RX CTCSS read-back"),
            )
        }
        if (kinds.first == CatRepeater.TONE_DCS || kinds.second == CatRepeater.TONE_DCS) {
            val dcs = state.dcs ?: return Result.failure(
                IllegalStateException("missing CI-V DCS read-back"),
            )
            if (kinds.first == CatRepeater.TONE_DCS) {
                txValue = dcs.first
                txPolarity = dcs.second
            }
            if (kinds.second == CatRepeater.TONE_DCS) {
                rxValue = dcs.first
                rxPolarity = dcs.third
            }
        }
        val config = CatRepeaterConfig(
            duplex = state.duplex,
            offsetHz = if (state.duplex == CatRepeater.DUPLEX_SIMPLEX) 0 else state.offsetHz,
            txKind = kinds.first,
            txValue = txValue,
            txPolarity = txPolarity,
            rxKind = kinds.second,
            rxValue = rxValue,
            rxPolarity = rxPolarity,
        )
        validateCivRepeater(config, repeaterProfile())?.let {
            return Result.failure(IllegalStateException(it))
        }
        return Result.success(config)
    }

    private fun readRepeaterConfig(cancellable: Boolean): Result<CatRepeaterConfig> =
        readRepeaterState(cancellable).fold(
            onSuccess = { repeaterConfigFromState(it) },
            onFailure = { Result.failure(it) },
        )

    private fun applyRepeaterConfig(config: CatRepeaterConfig): String? {
        val addr = rigAddr()
        val caps = CivModels.rigCaps(addr)
        validateCivRepeater(config, repeaterProfile())?.let { return it }

        // Make selectors inert first, write latent values, and activate last.
        when (addr) {
            CivModels.ADDR_IC705, CivModels.ADDR_IC9700, CivModels.ADDR_IC905 ->
                setFuncValue(P.SUB_FUNC_TONE_MODE, 0, true)?.let { return it }
            CivModels.ADDR_ICR8600 -> {
                setFuncValue(P.SUB_FUNC_TONE_SQUELCH, 0, true)?.let { return it }
                setFuncValue(P.SUB_FUNC_DCS, 0, true)?.let { return it }
            }
            else -> {
                setFuncValue(P.SUB_FUNC_REPEATER_TONE, 0, true)?.let { return it }
                setFuncValue(P.SUB_FUNC_TONE_SQUELCH, 0, true)?.let { return it }
            }
        }
        if (caps.repeaterCaps and CatRepeater.CAP_DUPLEX != 0) {
            val frame = P.setDuplex(addr, CatRepeater.DUPLEX_SIMPLEX)
                ?: return "cannot encode CI-V simplex selector"
            writeCommand(frame, P.CMD_DUPLEX, "neutralizing CI-V duplex", true)?.let {
                return it
            }
        }

        if (config.txKind == CatRepeater.TONE_CTCSS) {
            writeCtcss(P.SUB_TONE_TX, config.txValue, true)?.let { return it }
        }
        if (config.rxKind == CatRepeater.TONE_CTCSS) {
            writeCtcss(P.SUB_TONE_RX, config.rxValue, true)?.let { return it }
        }
        if (config.txKind == CatRepeater.TONE_DCS || config.rxKind == CatRepeater.TONE_DCS) {
            val code = if (config.txKind == CatRepeater.TONE_DCS) {
                config.txValue
            } else {
                config.rxValue
            }
            val frame = P.setDcs(addr, code, config.txPolarity, config.rxPolarity)
                ?: return "cannot encode CI-V DCS request"
            writeCommand(frame, P.CMD_TONE, "setting CI-V DCS", true)?.let { return it }
        }

        when (addr) {
            CivModels.ADDR_IC705, CivModels.ADDR_IC9700, CivModels.ADDR_IC905 -> {
                val mode = civCrossToneMode(config)
                    ?: return "CI-V cross-tone selector cannot represent the request"
                setFuncValue(P.SUB_FUNC_TONE_MODE, mode, true)?.let { return it }
            }
            CivModels.ADDR_ICR8600 -> when (config.rxKind) {
                CatRepeater.TONE_CTCSS ->
                    setFuncValue(P.SUB_FUNC_TONE_SQUELCH, 1, true)?.let { return it }
                CatRepeater.TONE_DCS ->
                    setFuncValue(P.SUB_FUNC_DCS, 1, true)?.let { return it }
            }
            else -> {
                if (config.rxKind == CatRepeater.TONE_CTCSS) {
                    setFuncValue(P.SUB_FUNC_TONE_SQUELCH, 1, true)?.let { return it }
                } else if (config.txKind == CatRepeater.TONE_CTCSS) {
                    setFuncValue(P.SUB_FUNC_REPEATER_TONE, 1, true)?.let { return it }
                }
            }
        }

        if (caps.repeaterCaps and CatRepeater.CAP_DUPLEX != 0) {
            if (config.duplex != CatRepeater.DUPLEX_SIMPLEX) {
                val frame = P.setRepeaterOffset(addr, config.offsetHz, caps.repeaterOffsetBytes)
                    ?: return "cannot encode CI-V repeater offset"
                writeCommand(
                    frame,
                    P.CMD_SET_REPEATER_OFFSET,
                    "setting CI-V repeater offset",
                    true,
                )?.let { return it }
            }
            val frame = P.setDuplex(addr, config.duplex)
                ?: return "cannot encode CI-V duplex direction"
            writeCommand(frame, P.CMD_DUPLEX, "setting CI-V duplex direction", true)?.let {
                return it
            }
        }
        return null
    }

    private fun restoreRepeaterState(state: CivRepeaterState): String? {
        val addr = rigAddr()
        val caps = CivModels.rigCaps(addr)
        val errors = ArrayList<String>()
        fun record(field: String, error: String?) {
            if (error != null) errors.add("$field: $error")
        }

        when (addr) {
            CivModels.ADDR_IC705, CivModels.ADDR_IC9700, CivModels.ADDR_IC905 ->
                record("neutral tone mode", setFuncValue(P.SUB_FUNC_TONE_MODE, 0, false))
            CivModels.ADDR_ICR8600 -> {
                record(
                    "neutral tone squelch",
                    setFuncValue(P.SUB_FUNC_TONE_SQUELCH, 0, false),
                )
                record("neutral DCS", setFuncValue(P.SUB_FUNC_DCS, 0, false))
            }
            else -> {
                record(
                    "neutral repeater tone",
                    setFuncValue(P.SUB_FUNC_REPEATER_TONE, 0, false),
                )
                record(
                    "neutral tone squelch",
                    setFuncValue(P.SUB_FUNC_TONE_SQUELCH, 0, false),
                )
            }
        }
        if (caps.repeaterCaps and CatRepeater.CAP_DUPLEX != 0) {
            record(
                "neutral duplex",
                P.setDuplex(addr, CatRepeater.DUPLEX_SIMPLEX)?.let {
                    writeCommand(it, P.CMD_DUPLEX, "setting simplex", false)
                } ?: "cannot encode simplex",
            )
        }
        state.txCtcss?.let {
            record("TX CTCSS", writeCtcss(P.SUB_TONE_TX, it, false))
        }
        state.rxCtcss?.let {
            record("RX CTCSS", writeCtcss(P.SUB_TONE_RX, it, false))
        }
        state.dcs?.let {
            val frame = P.setDcs(addr, it.first, it.second, it.third)
            record(
                "DCS",
                frame?.let { encoded ->
                    writeCommand(encoded, P.CMD_TONE, "restoring CI-V DCS", false)
                } ?: "cannot encode saved CI-V DCS state",
            )
        }
        if (caps.repeaterCaps and CatRepeater.CAP_OFFSET != 0) {
            val frame = P.setRepeaterOffset(addr, state.offsetHz, caps.repeaterOffsetBytes)
            record(
                "offset",
                frame?.let {
                    writeCommand(
                        it,
                        P.CMD_SET_REPEATER_OFFSET,
                        "restoring CI-V repeater offset",
                        false,
                    )
                } ?: "cannot encode saved CI-V repeater offset",
            )
        }
        state.toneMode?.let {
            record("tone mode", setFuncValue(P.SUB_FUNC_TONE_MODE, it, false))
        }
        state.repeaterToneEnabled?.let {
            record(
                "repeater tone selector",
                setFuncValue(P.SUB_FUNC_REPEATER_TONE, it, false),
            )
        }
        state.toneSquelchEnabled?.let {
            record(
                "tone squelch selector",
                setFuncValue(P.SUB_FUNC_TONE_SQUELCH, it, false),
            )
        }
        state.dcsEnabled?.let {
            record("DCS selector", setFuncValue(P.SUB_FUNC_DCS, it, false))
        }
        if (caps.repeaterCaps and CatRepeater.CAP_DUPLEX != 0) {
            val frame = P.setDuplex(addr, state.duplex)
            record(
                "duplex",
                frame?.let {
                    writeCommand(it, P.CMD_DUPLEX, "restoring CI-V duplex direction", false)
                } ?: "cannot encode saved CI-V duplex direction",
            )
        }
        readRepeaterState(false).fold(
            onSuccess = {
                if (it != state) {
                    errors.add("final snapshot mismatch: expected $state, read $it")
                }
            },
            onFailure = {
                errors.add("final snapshot unreadable: ${it.message ?: "unknown error"}")
            },
        )
        return errors.takeIf { it.isNotEmpty() }?.joinToString("; ")
    }

    private fun setCatRepeaterActive(config: CatRepeaterConfig): String? {
        if (repeaterUncertain) {
            return "CI-V repeater state is uncertain after a failed rollback; " +
                "reconnect before changing it"
        }
        validateCivRepeater(config, repeaterProfile())?.let { return it }
        val physicalPtt = readPhysicalPtt(true).getOrElse {
            return it.message ?: "cannot read physical CI-V PTT state"
        }
        if (physicalPtt) return "cannot change CI-V repeater state while transmitting"
        confirmPhysicalFm(true)?.let { return it }
        val previous = readRepeaterState(true).getOrElse {
            return it.message ?: "cannot read physical CI-V repeater state"
        }
        activeRepeater?.let { expected ->
            val actual = repeaterConfigFromState(previous).getOrElse {
                return it.message ?: "cannot decode physical CI-V repeater state"
            }
            if (actual != expected) {
                return "CI-V repeater state changed outside iSDR: expected $expected, read $actual"
            }
        }

        repeaterMutationStarted = true
        var error = applyRepeaterConfig(config)
        if (error == null) error = confirmPhysicalFm(true)
        if (error == null) {
            val actual = readRepeaterConfig(true).getOrElse {
                error = it.message ?: "cannot confirm physical CI-V repeater state"
                null
            }
            if (actual != null && actual != config) {
                error = "CI-V repeater confirmation mismatch: requested $config, read $actual"
            }
        }
        if (error == null) {
            activeRepeater = config
            return null
        }
        if (repeaterCancel.get()) {
            repeaterUncertain = true
            return "$error; CI-V repeater transaction was cancelled for priority unkey and " +
                "the session is now fail-closed"
        }
        val rollback = restoreRepeaterState(previous)
        return if (rollback == null) {
            "$error; previous CI-V repeater state was restored"
        } else {
            repeaterUncertain = true
            "$error; CI-V repeater rollback failed and the session is now fail-closed: $rollback"
        }
    }

    private fun probeAddr(): Int? {
        for (addr in PROBE_ADDRS) {
            rigAddrAtomic.set(addr)
            val data = transact(P.readTransceiverId(addr), P.CMD_READ_ID) ?: continue
            // Reply data: sub-command echo then the rig's address.
            if (data.size >= 2 && (data[0].toInt() and 0xFF) == P.SUB_ID) {
                return data[1].toInt() and 0xFF
            }
            return addr
        }
        return null
    }

    private fun readInitialState() {
        val addr = rigAddr()
        transact(P.readFrequency(addr), P.CMD_READ_FREQ)?.let { data ->
            if (data.size >= 5) {
                P.fromBcdLe(data.copyOfRange(0, 5))?.let { freqHz.set(it) }
            }
        }
        transact(P.readMode(addr), P.CMD_READ_MODE)?.let { data ->
            if (data.isNotEmpty()) modeCode.set(data[0].toInt() and 0xFF)
        }
    }

    private fun enableScope() {
        val addr = rigAddr()
        // Scope on, waveform routed to this port; a control-only rig NAKs
        // both and simply stays a CAT radio.
        transact(P.scopeOn(addr, true), P.CMD_SCOPE)
        transact(P.scopeWaveOutput(addr, true), P.CMD_SCOPE)
    }

    // ---- reader ---------------------------------------------------------------

    private fun readerLoop(transport: CivTransport) {
        val deframer = P.Deframer()
        var assembler: P.ScopeAssembler? = null
        var assemblerCaps: CivModels.ScopeCaps? = null
        val buf = ByteArray(4096)
        try {
            while (running.get()) {
                while (true) {
                    val frame = outgoing.poll() ?: break
                    try {
                        transport.writeAll(frame.bytes)
                    } finally {
                        frame.written.countDown()
                    }
                }
                val n = transport.readSome(buf)
                if (n > 0) {
                    for (body in deframer.push(buf, n)) {
                        val frame = P.parseFrame(body) ?: continue
                        if (P.isEcho(frame, P.CONTROLLER_ADDR)) continue
                        if (assembler == null) {
                            val caps = CivModels.scopeCaps(rigAddr())
                                ?: CivModels.ScopeCaps.standard()
                            assembler = P.ScopeAssembler(caps.lineLength)
                            assemblerCaps = caps
                        }
                        handleFrame(frame, assembler!!, assemblerCaps!!)
                    }
                }
                readerCycle.incrementAndGet()
            }
        } catch (error: Exception) {
            if (running.getAndSet(false)) {
                onConnectionStatusChanged(false, "link lost")
            }
            failPending(error)
        } finally {
            while (true) outgoing.poll()?.written?.countDown() ?: break
            transport.close()
        }
    }

    private fun handleFrame(
        frame: P.Frame,
        assembler: P.ScopeAssembler,
        caps: CivModels.ScopeCaps,
    ) {
        when (frame) {
            is P.Frame.Ack -> completePending(frame.from) { Result.success(ByteArray(0)) }
            is P.Frame.Nak -> completePending(frame.from) {
                Result.failure(IllegalStateException("rig rejected the command"))
            }
            is P.Frame.Message -> {
                val data = frame.data
                when {
                    frame.cmd == P.CMD_TRANSCEIVE_FREQ -> {
                        if (data.size >= 5) {
                            P.fromBcdLe(data.copyOfRange(0, 5))?.let { freqHz.set(it) }
                        }
                        return
                    }
                    frame.cmd == P.CMD_TRANSCEIVE_MODE -> {
                        if (data.isNotEmpty()) modeCode.set(data[0].toInt() and 0xFF)
                        return
                    }
                    frame.cmd == P.CMD_SCOPE && data.isNotEmpty() &&
                        (data[0].toInt() and 0xFF) == P.SUB_SCOPE_WAVE -> {
                        handleScope(assembler, caps, data.copyOfRange(1, data.size))
                        return
                    }
                }
                // Solicited reply: the rig echoes the command byte it answers.
                synchronized(pendingLock) {
                    val p = pending
                    if (p != null && p.cmd == frame.cmd && p.rigAddr == frame.from) {
                        pending = null
                        p.reply.offer(Result.success(data))
                    }
                }
            }
        }
    }

    private inline fun completePending(from: Int, result: () -> Result<ByteArray>) {
        synchronized(pendingLock) {
            val p = pending ?: return
            if (p.rigAddr != from) return
            pending = null
            p.reply.offer(result())
        }
    }

    private fun failPending(error: Exception) {
        synchronized(pendingLock) {
            val p = pending ?: return
            pending = null
            p.reply.offer(Result.failure(error))
        }
    }

    private fun handleScope(
        assembler: P.ScopeAssembler,
        caps: CivModels.ScopeCaps,
        data: ByteArray,
    ) {
        val line = assembler.push(data) ?: return
        // Sub-scope sweeps would need their own display plane; main only.
        if (line.id != 0) return
        val spanChanged = spanHz.getAndSet(line.spanHz()) != line.spanHz()
        lowEdgeHz.set(line.lowEdgeHz)
        highEdgeHz.set(line.highEdgeHz)
        // A changed span is a changed effective rate: let the host announce
        // the truth of the sweep header rather than the request.
        if (spanChanged) stateListener?.invoke()
        if (!spectrumWanted) return
        val spectrum = P.binsToDb(line.bins, caps.levelMax, caps.dbMin, caps.dbMax)
        onDataReceived(spectrum, EMPTY_FLOATS)
    }

    // ---- RadioClient -----------------------------------------------------------

    override suspend fun connect(): Boolean = withContext(Dispatchers.IO) { connectBlocking() }

    /** Blocking body of [connect]; also directly testable off a coroutine. */
    fun connectBlocking(): Boolean {
        val t: CivTransport
        synchronized(this) {
            if (started) return running.get()
            started = true
            t = transport ?: return false
            transport = null
        }
        running.set(true)
        reader = thread(name = "civ-reader") { readerLoop(t) }

        val addr = if (configuredAddr != 0) {
            configuredAddr
        } else {
            probeAddr() ?: run {
                disconnect()
                onConnectionStatusChanged(false, "no CI-V rig answered")
                return false
            }
        }
        rigAddrAtomic.set(addr)

        // A configured address still has to answer before this counts as
        // connected; a silent port is a failure, not a session.
        if (configuredAddr != 0 &&
            transact(P.readTransceiverId(addr), P.CMD_READ_ID) == null
        ) {
            disconnect()
            onConnectionStatusChanged(false, "rig did not answer")
            return false
        }

        scopeCaps = CivModels.scopeCaps(addr)
        readInitialState()
        if (scopeCaps != null) enableScope()
        onConnectionStatusChanged(true, modelName())
        return true
    }

    override fun disconnect() {
        if (running.getAndSet(false)) {
            reader?.join()
            reader = null
        }
        // Never started: the transport is still ours to release.
        synchronized(this) {
            transport?.close()
            transport = null
        }
    }

    override fun setFrequency(hz: Long) {
        repeaterMutationGuard("tune")?.let { throw IllegalStateException(it) }
        val addr = rigAddr()
        val expectedRepeater = activeRepeater
        expectedRepeater?.let { expected ->
            validateCivRepeater(expected, civRepeaterProfile(addr, hz))?.let {
                throw IllegalStateException("tune refused at $hz Hz: $it")
            }
        }
        val frame = P.writeFrequency(addr, hz)
            ?: throw IllegalArgumentException("CI-V frequency $hz Hz is not encodable")
        writeCommand(frame, P.CMD_WRITE_FREQ, "CI-V tune to $hz Hz", false)?.let {
            throw IllegalStateException(it)
        }
        val data = transactNamed(
            P.readFrequency(addr),
            P.CMD_READ_FREQ,
            "reading physical CI-V frequency",
            false,
        ).getOrElse { throw IllegalStateException(it.message ?: "frequency read-back failed") }
        if (data.size != 5) {
            throw IllegalStateException(
                "malformed CI-V frequency read-back ${wireBytes(data)}",
            )
        }
        val actual = P.fromBcdLe(data)
            ?: throw IllegalStateException("non-BCD CI-V frequency read-back ${wireBytes(data)}")
        freqHz.set(actual)
        if (actual != hz) {
            throw IllegalStateException(
                "CI-V frequency mismatch: requested $hz Hz, physical read-back was $actual Hz",
            )
        }
        expectedRepeater?.let { expected ->
            confirmPhysicalFm(false)?.let {
                repeaterUncertain = true
                throw IllegalStateException("post-tune repeater confirmation failed: $it")
            }
            val actualRepeater = readRepeaterConfig(false).getOrElse {
                repeaterUncertain = true
                throw IllegalStateException(
                    "post-tune repeater confirmation failed: " +
                        (it.message ?: "cannot read repeater state"),
                )
            }
            if (actualRepeater != expected) {
                repeaterUncertain = true
                throw IllegalStateException(
                    "post-tune CI-V repeater mismatch: expected $expected, read $actualRepeater; " +
                        "session is now fail-closed",
                )
            }
        }
    }

    override fun frequencyHz(): Long = freqHz.get()

    override fun setSampleRate(hz: Int) {
        // The "rate" of a scope-only stream is its span. The app already
        // chooses from the exact model ladder; silently snapping here would
        // make its axis disagree with the physical scope.
        val caps = scopeCaps ?: throw IllegalStateException("CI-V rig has no scope span control")
        val span = hz.toLong()
        if (span !in caps.spansHz) {
            throw IllegalArgumentException("CI-V scope span $span Hz is unsupported by ${modelName()}")
        }
        val frame = P.scopeSetSpan(rigAddr(), 0, span)
            ?: throw IllegalArgumentException("CI-V scope span $span Hz is not encodable")
        if (transact(frame, P.CMD_SCOPE) == null) {
            throw IllegalStateException("CI-V scope span $span Hz was not confirmed")
        }
        spanHz.set(span)
    }

    override fun sampleRateHz(): Int = spanHz.get().toInt()

    // ---- TransmitCapable ---------------------------------------------------------

    override fun setTxFrequency(hz: Long): Boolean {
        // A plain CI-V frequency write targets the active receive VFO. Until
        // this dialect has model-gated split/duplex commands plus read-back,
        // claiming a TX-frequency write would silently move reception.
        return false
    }

    override fun setPtt(on: Boolean) {
        val addr = rigAddr()
        val caps = CivModels.rigCaps(addr)
        if (!caps.hasTx) {
            ptt.set(false)
            if (on) throw IllegalStateException("${modelName()} is receive-only")
            return
        }
        if (on) {
            repeaterMutationGuard("PTT key")?.let { throw IllegalStateException(it) }
        }
        val setError = writeCommand(
            P.setPtt(addr, on),
            P.CMD_PTT,
            "CI-V PTT ${if (on) "key" else "unkey"}",
            false,
        )
        var error = setError
        if (error == null) {
            val actual = readPhysicalPtt().getOrElse {
                error = it.message ?: "cannot read physical CI-V PTT state"
                null
            }
            if (actual != null && actual == on) {
                ptt.set(actual)
                return
            }
            if (actual != null) {
                error = "CI-V PTT mismatch: requested ${if (on) "TX" else "RX"}, " +
                    "physical read-back was ${if (actual) "TX" else "RX"}"
            }
        }
        if (!on) throw IllegalStateException(error ?: "CI-V unkey was not confirmed")

        // Ambiguous keying is made safe before failure is reported.
        val unkeyError = writeCommand(
            P.setPtt(addr, false),
            P.CMD_PTT,
            "failsafe CI-V unkey",
            false,
        )
        val failsafeError = if (unkeyError != null) {
            unkeyError
        } else {
            readPhysicalPtt().fold(
                onSuccess = { if (it) "physical PTT remained keyed" else null },
                onFailure = { "unkey read-back failed: ${it.message ?: "unknown error"}" },
            )
        }
        if (failsafeError == null) {
            ptt.set(false)
            throw IllegalStateException("$error; failsafe unkey was physically confirmed")
        }
        repeaterUncertain = true
        throw IllegalStateException(
            "$error; failsafe unkey could not be confirmed and the session is now " +
                "fail-closed: $failsafeError",
        )
    }

    private fun repeaterMutationGuard(action: String): String? {
        if (repeaterInFlight.get()) {
            return "$action refused while a CI-V repeater transaction is active"
        }
        if (repeaterUncertain) {
            return "CI-V repeater/PTT state is uncertain after a failed rollback; " +
                "$action refused until reconnect"
        }
        activeRepeater?.let { expected ->
            confirmPhysicalFm(false)?.let { return "$action refused: $it" }
            val actual = readRepeaterConfig(false).getOrElse {
                return "$action refused: ${it.message ?: "cannot read repeater state"}"
            }
            if (actual != expected) {
                return "CI-V repeater state changed outside iSDR; $action refused: " +
                    "expected $expected, read $actual"
            }
        }
        return null
    }

    override fun submitTxIq(iq: FloatArray) {
        // TX audio rides the rig's own soundcard plane; CI-V carries only
        // the keying.
    }

    override fun isTransmitting(): Boolean = ptt.get()
}
