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

import com.isaklab.isdrdrivers.core.RadioClient
import com.isaklab.isdrdrivers.core.CatControlCapable
import com.isaklab.isdrdrivers.core.TransmitCapable
import com.isaklab.libcivk.CivProtocol as P
import java.util.concurrent.ArrayBlockingQueue
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
) : RadioClient, TransmitCapable, CatControlCapable {

    companion object {
        /** How long one request waits for its reply before retrying, in ms. */
        private const val REPLY_TIMEOUT_MS = 300L

        /** Attempts per request; the bus is multi-drop and collisions are normal. */
        private const val ATTEMPTS = 3

        private val EMPTY_FLOATS = FloatArray(0)

        /**
         * Spans the centre-mode scope accepts, in Hz. Requests snap to the
         * nearest; the rig's read-back remains the truth.
         */
        val SCOPE_SPANS = longArrayOf(
            5_000, 10_000, 25_000, 50_000, 100_000, 250_000, 500_000, 1_000_000,
        )

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

    private class Pending(val cmd: Int) {
        /** null data = NAK / rejected. */
        val reply = ArrayBlockingQueue<Result<ByteArray>>(1)
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

    private val outgoing = LinkedBlockingQueue<ByteArray>()
    private val pendingLock = Object()
    private var pending: Pending? = null

    private var reader: Thread? = null
    private var started = false
    private var scopeCaps: CivModels.ScopeCaps? = null
    @Volatile private var stateListener: (() -> Unit)? = null

    @Volatile private var spectrumWanted = true

    override var spectrumEnabled: Boolean
        get() = spectrumWanted
        set(value) {
            spectrumWanted = value
            // Also stop the rig streaming sweeps nobody will draw. The only
            // reply is an ACK the reader consumes anyway, so fire-and-forget.
            if (scopeCaps != null) outgoing.offer(P.scopeWaveOutput(rigAddr(), value))
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
    private fun transact(frame: ByteArray, cmd: Int): ByteArray? {
        if (!running.get()) return null
        repeat(ATTEMPTS) {
            val p = Pending(cmd)
            synchronized(pendingLock) { pending = p }
            outgoing.offer(frame)
            val result = p.reply.poll(REPLY_TIMEOUT_MS, TimeUnit.MILLISECONDS)
            synchronized(pendingLock) { if (pending === p) pending = null }
            if (result != null) return result.getOrNull()
        }
        return null
    }

    // ---- rig control --------------------------------------------------------

    /** The rig IF filter (FIL1..3) the last mode write carried. */
    private var lastFil = 1

    /**
     * Set the operating mode (CI-V mode code), carrying the rig IF filter
     * selection the last CATCTL_FIL chose (FIL1 on a fresh session).
     */
    fun setMode(mode: Int): Boolean {
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
            else -> false
        }
    }

    override fun setCatMode(mode: Int): Boolean = setMode(mode)

    override fun currentCatMode(): Int = mode()

    override fun setCatControl(id: Int, value: Int): Boolean = setControl(id, value)

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
                    transport.writeAll(frame)
                }
                val n = transport.readSome(buf)
                if (n == 0) continue
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
        } catch (_: Exception) {
            if (running.getAndSet(false)) {
                onConnectionStatusChanged(false, "link lost")
            }
        } finally {
            transport.close()
        }
    }

    private fun handleFrame(
        frame: P.Frame,
        assembler: P.ScopeAssembler,
        caps: CivModels.ScopeCaps,
    ) {
        when (frame) {
            is P.Frame.Ack -> completePending { Result.success(ByteArray(0)) }
            is P.Frame.Nak -> completePending {
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
                    if (p != null && p.cmd == frame.cmd) {
                        pending = null
                        p.reply.offer(Result.success(data))
                    }
                }
            }
        }
    }

    private inline fun completePending(result: () -> Result<ByteArray>) {
        synchronized(pendingLock) {
            val p = pending ?: return
            pending = null
            p.reply.offer(result())
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
        val frame = P.writeFrequency(rigAddr(), hz) ?: return
        if (transact(frame, P.CMD_WRITE_FREQ) != null) freqHz.set(hz)
    }

    override fun frequencyHz(): Long = freqHz.get()

    override fun setSampleRate(hz: Int) {
        // The "rate" of a scope-only stream is its span. Snap the request to
        // a span the scope accepts, then trust only the sweep headers to
        // report what is actually in force.
        if (scopeCaps == null) return
        val target = hz.toLong()
        val span = SCOPE_SPANS.minByOrNull { kotlin.math.abs(it - target) }!!
        val frame = P.scopeSetSpan(rigAddr(), 0, span) ?: return
        if (transact(frame, P.CMD_SCOPE) != null) spanHz.set(span)
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
        if (transact(P.setPtt(rigAddr(), on), P.CMD_PTT) != null) ptt.set(on)
    }

    override fun submitTxIq(iq: FloatArray) {
        // TX audio rides the rig's own soundcard plane; CI-V carries only
        // the keying.
    }

    override fun isTransmitting(): Boolean = ptt.get()
}
