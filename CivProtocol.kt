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

/**
 * Frame codec for the Icom CI-V bus.
 *
 * Builds controller-to-rig frames and parses rig-to-controller frames. Holds
 * no I/O and no clock; the caller owns the port, the retry policy and the
 * scope-line assembly state, so every byte of the wire format is
 * unit-testable without a rig.
 *
 * Wire format: `FE FE <to> <from> <cmd> [<sub>] [data...] FD`. The data area
 * never contains `0xFC..0xFE`, which is what makes resynchronisation on the
 * preamble safe without an escape scheme.
 */
object CivProtocol {

    /** Preamble byte, sent twice before every frame. */
    const val PREAMBLE = 0xFE

    /** Frame terminator. */
    const val TERMINATOR = 0xFD

    /**
     * Bus collision marker: a rig that detects a clash jams the line with
     * `0xFC`; everything received since the last terminator is garbage.
     */
    const val COLLISION = 0xFC

    /** Command byte of a positive acknowledge frame. */
    const val ACK = 0xFB

    /** Command byte of a negative acknowledge frame. */
    const val NAK = 0xFA

    /** Our own bus address as the controller. */
    const val CONTROLLER_ADDR = 0xE0

    /** Destination of unsolicited (transceive/scope) frames. */
    const val BROADCAST_ADDR = 0x00

    /**
     * Longest legal frame body this codec will accept between the preamble
     * and the terminator. Scope data frames on current rigs stay well under
     * this; anything longer is line noise that swallowed a terminator.
     */
    const val MAX_BODY = 256

    // ---- commands -----------------------------------------------------------

    const val CMD_TRANSCEIVE_FREQ = 0x00
    const val CMD_TRANSCEIVE_MODE = 0x01
    const val CMD_READ_FREQ = 0x03
    const val CMD_READ_MODE = 0x04
    const val CMD_WRITE_FREQ = 0x05
    const val CMD_WRITE_MODE = 0x06
    const val CMD_LEVEL = 0x14
    const val CMD_READ_METER = 0x15
    const val CMD_READ_ID = 0x19
    const val CMD_PTT = 0x1C
    const val CMD_SCOPE = 0x27

    const val SUB_LEVEL_AF = 0x01
    const val SUB_LEVEL_RF = 0x02
    const val SUB_LEVEL_RFPOWER = 0x0A
    const val SUB_METER_S = 0x02
    const val SUB_METER_POWER = 0x11
    const val SUB_METER_SWR = 0x12
    const val SUB_METER_ALC = 0x13
    const val SUB_PTT = 0x00
    const val SUB_ID = 0x00

    const val SUB_SCOPE_WAVE = 0x00
    const val SUB_SCOPE_ON = 0x10
    const val SUB_SCOPE_WAVE_OUTPUT = 0x11
    const val SUB_SCOPE_MODE = 0x14
    const val SUB_SCOPE_SPAN = 0x15
    const val SUB_SCOPE_REF = 0x19
    const val SUB_SCOPE_SPEED = 0x1A
    const val SUB_SCOPE_FIXED_EDGES = 0x1E

    /** Operating-mode codes (command 0x01/0x04/0x06 data byte). */
    const val MODE_LSB = 0x00
    const val MODE_USB = 0x01
    const val MODE_AM = 0x02
    const val MODE_CW = 0x03
    const val MODE_RTTY = 0x04
    const val MODE_FM = 0x05
    const val MODE_CW_R = 0x07
    const val MODE_RTTY_R = 0x08

    /** Highest frequency expressible in the 10-digit BCD field. */
    const val MAX_FREQ_HZ = 9_999_999_999L

    const val SCOPE_MODE_CENTER = 0x00
    const val SCOPE_MODE_FIXED = 0x01

    // ---- BCD ------------------------------------------------------------------

    /**
     * Pack [value] as packed BCD, least-significant byte first, two digits
     * per byte. Null when the value does not fit in `2 * len` digits.
     */
    fun toBcdLe(value: Long, len: Int): ByteArray? {
        if (value < 0) return null
        var v = value
        val out = ByteArray(len)
        for (i in 0 until len) {
            val lo = (v % 10).toInt()
            v /= 10
            val hi = (v % 10).toInt()
            v /= 10
            out[i] = ((hi shl 4) or lo).toByte()
        }
        if (v != 0L) return null
        return out
    }

    /**
     * Decode packed little-endian BCD. Null on any nibble above 9 — a frame
     * whose digits are not digits is corrupt, not approximately right.
     */
    fun fromBcdLe(bytes: ByteArray): Long? {
        var v = 0L
        for (i in bytes.indices.reversed()) {
            val b = bytes[i].toInt() and 0xFF
            val hi = b shr 4
            val lo = b and 0x0F
            if (hi > 9 || lo > 9) return null
            v = v * 100 + hi * 10 + lo
        }
        return v
    }

    /** Decode packed big-endian BCD (levels, meters, scope division counters). */
    fun fromBcdBe(bytes: ByteArray): Long? {
        var v = 0L
        for (i in bytes.indices) {
            val b = bytes[i].toInt() and 0xFF
            val hi = b shr 4
            val lo = b and 0x0F
            if (hi > 9 || lo > 9) return null
            v = v * 100 + hi * 10 + lo
        }
        return v
    }

    /** Pack [value] (0..9999) as 2-byte big-endian BCD — the level format. */
    fun toBcdBe2(value: Int): ByteArray? {
        if (value !in 0..9999) return null
        val d0 = value / 1000
        val d1 = value / 100 % 10
        val d2 = value / 10 % 10
        val d3 = value % 10
        return byteArrayOf(((d0 shl 4) or d1).toByte(), ((d2 shl 4) or d3).toByte())
    }

    // ---- frame building ---------------------------------------------------------

    /**
     * Wrap [body] (command byte onward) into a full frame. Null when the
     * body is empty, oversized, or contains a byte the data area cannot
     * carry (`0xFC..0xFE`), which would desynchronise every listener on the
     * bus.
     */
    fun buildFrame(to: Int, from: Int, body: ByteArray): ByteArray? {
        if (body.isEmpty() || body.size > MAX_BODY) return null
        if (body.any { (it.toInt() and 0xFF) >= COLLISION }) return null
        if (to >= COLLISION || from >= COLLISION) return null
        val f = ByteArray(body.size + 5)
        f[0] = PREAMBLE.toByte()
        f[1] = PREAMBLE.toByte()
        f[2] = to.toByte()
        f[3] = from.toByte()
        body.copyInto(f, 4)
        f[f.size - 1] = TERMINATOR.toByte()
        return f
    }

    fun readFrequency(to: Int): ByteArray =
        buildFrame(to, CONTROLLER_ADDR, byteArrayOf(CMD_READ_FREQ.toByte()))!!

    /** Null when [hz] is negative or beyond the 10-digit field. */
    fun writeFrequency(to: Int, hz: Long): ByteArray? {
        if (hz !in 0..MAX_FREQ_HZ) return null
        val bcd = toBcdLe(hz, 5) ?: return null
        return buildFrame(to, CONTROLLER_ADDR, byteArrayOf(CMD_WRITE_FREQ.toByte()) + bcd)
    }

    fun readMode(to: Int): ByteArray =
        buildFrame(to, CONTROLLER_ADDR, byteArrayOf(CMD_READ_MODE.toByte()))!!

    /** [filter] is the rig's passband selection 1..3. */
    fun writeMode(to: Int, mode: Int, filter: Int): ByteArray? {
        if (filter !in 1..3) return null
        return buildFrame(
            to, CONTROLLER_ADDR,
            byteArrayOf(CMD_WRITE_MODE.toByte(), mode.toByte(), filter.toByte()),
        )
    }

    fun setPtt(to: Int, on: Boolean): ByteArray =
        buildFrame(
            to, CONTROLLER_ADDR,
            byteArrayOf(CMD_PTT.toByte(), SUB_PTT.toByte(), if (on) 1 else 0),
        )!!

    fun readPtt(to: Int): ByteArray =
        buildFrame(to, CONTROLLER_ADDR, byteArrayOf(CMD_PTT.toByte(), SUB_PTT.toByte()))!!

    fun readTransceiverId(to: Int): ByteArray =
        buildFrame(to, CONTROLLER_ADDR, byteArrayOf(CMD_READ_ID.toByte(), SUB_ID.toByte()))!!

    /** [value] on the rig's 0..255 scale, sent as 4-digit big-endian BCD. */
    fun setLevel(to: Int, sub: Int, value: Int): ByteArray {
        val bcd = toBcdBe2(value and 0xFF)!!
        return buildFrame(
            to, CONTROLLER_ADDR,
            byteArrayOf(CMD_LEVEL.toByte(), sub.toByte(), bcd[0], bcd[1]),
        )!!
    }

    fun readLevel(to: Int, sub: Int): ByteArray =
        buildFrame(to, CONTROLLER_ADDR, byteArrayOf(CMD_LEVEL.toByte(), sub.toByte()))!!

    fun readMeter(to: Int, sub: Int): ByteArray =
        buildFrame(to, CONTROLLER_ADDR, byteArrayOf(CMD_READ_METER.toByte(), sub.toByte()))!!

    fun scopeOn(to: Int, on: Boolean): ByteArray =
        buildFrame(
            to, CONTROLLER_ADDR,
            byteArrayOf(CMD_SCOPE.toByte(), SUB_SCOPE_ON.toByte(), if (on) 1 else 0),
        )!!

    /**
     * Route the scope waveform to the CI-V port instead of only the rig's
     * own display.
     */
    fun scopeWaveOutput(to: Int, on: Boolean): ByteArray =
        buildFrame(
            to, CONTROLLER_ADDR,
            byteArrayOf(CMD_SCOPE.toByte(), SUB_SCOPE_WAVE_OUTPUT.toByte(), if (on) 1 else 0),
        )!!

    /** Main ([id] 0) or sub ([id] 1) scope, centre or fixed mode. */
    fun scopeSetMode(to: Int, id: Int, mode: Int): ByteArray? {
        if (id > 1 || id < 0 || mode !in 0..0x03) return null
        return buildFrame(
            to, CONTROLLER_ADDR,
            byteArrayOf(CMD_SCOPE.toByte(), SUB_SCOPE_MODE.toByte(), id.toByte(), mode.toByte()),
        )
    }

    /**
     * Centre-mode span of scope [id], in Hz. The wire carries the HALF-span
     * (the +/- deviation from centre); the rig only accepts its own discrete
     * steps, so the caller reads back what actually stuck.
     */
    fun scopeSetSpan(to: Int, id: Int, spanHz: Long): ByteArray? {
        if (id > 1 || id < 0 || spanHz !in 0..MAX_FREQ_HZ) return null
        val bcd = toBcdLe(spanHz / 2, 5) ?: return null
        return buildFrame(
            to, CONTROLLER_ADDR,
            byteArrayOf(CMD_SCOPE.toByte(), SUB_SCOPE_SPAN.toByte(), id.toByte()) + bcd,
        )
    }

    fun readScopeSpan(to: Int, id: Int): ByteArray? {
        if (id > 1 || id < 0) return null
        return buildFrame(
            to, CONTROLLER_ADDR,
            byteArrayOf(CMD_SCOPE.toByte(), SUB_SCOPE_SPAN.toByte(), id.toByte()),
        )
    }

    /**
     * Reference level of scope [id], in half-dB steps (-20.0..+20.0 dB): the
     * magnitude travels as 4-digit BCD centi-dB, then a sign byte.
     */
    fun scopeSetRef(to: Int, id: Int, dbTimes2: Int): ByteArray? {
        if (id > 1 || id < 0 || dbTimes2 !in -40..40) return null
        val centiDb = kotlin.math.abs(dbTimes2) * 50
        val bcd = toBcdBe2(centiDb) ?: return null
        val sign: Byte = if (dbTimes2 < 0) 0x01 else 0x00
        return buildFrame(
            to, CONTROLLER_ADDR,
            byteArrayOf(
                CMD_SCOPE.toByte(), SUB_SCOPE_REF.toByte(), id.toByte(), bcd[0], bcd[1], sign,
            ),
        )
    }

    // ---- deframing ----------------------------------------------------------

    /** What one received frame means to the controller. */
    sealed class Frame {
        /** `FE FE <to> <from> FB FD` */
        data class Ack(val from: Int) : Frame()

        /** `FE FE <to> <from> FA FD` */
        data class Nak(val from: Int) : Frame()

        /** Solicited reply or unsolicited transceive data. */
        class Message(val to: Int, val from: Int, val cmd: Int, val data: ByteArray) : Frame()
    }

    /**
     * Splits the raw byte stream into frames: resynchronises on the double
     * preamble, terminates on `FD`, discards everything pending when a
     * collision marker appears, and bounds the body so a lost terminator
     * cannot grow the buffer without limit.
     */
    class Deframer {
        private val buf = ArrayList<Byte>(MAX_BODY)
        private var inFrame = false

        /** Collisions observed since construction — the client's retry trigger. */
        var collisions = 0L
            private set

        var oversizeDrops = 0L
            private set

        /**
         * Feed [length] received bytes; returns every complete frame body
         * they closed, each starting at the `<to>` address byte (preamble
         * and terminator stripped).
         */
        fun push(bytes: ByteArray, length: Int = bytes.size): List<ByteArray> {
            val out = ArrayList<ByteArray>()
            for (i in 0 until length) {
                when (bytes[i].toInt() and 0xFF) {
                    COLLISION -> {
                        buf.clear()
                        inFrame = false
                        collisions++
                    }
                    PREAMBLE -> {
                        // A preamble inside a frame can only mean the previous
                        // terminator was lost: restart, keeping frame state.
                        if (inFrame && buf.isNotEmpty()) buf.clear()
                        inFrame = true
                    }
                    TERMINATOR -> {
                        if (inFrame && buf.size >= 3) {
                            out.add(buf.toByteArray())
                        }
                        buf.clear()
                        inFrame = false
                    }
                    else -> {
                        if (inFrame) {
                            buf.add(bytes[i])
                            if (buf.size > MAX_BODY) {
                                buf.clear()
                                inFrame = false
                                oversizeDrops++
                            }
                        }
                    }
                }
            }
            return out
        }
    }

    /**
     * Interpret one deframed body. Null for garbage (too short — a body
     * needs at least both addresses and a command).
     */
    fun parseFrame(body: ByteArray): Frame? {
        if (body.size < 3) return null
        val to = body[0].toInt() and 0xFF
        val from = body[1].toInt() and 0xFF
        return when (val cmd = body[2].toInt() and 0xFF) {
            ACK -> Frame.Ack(from)
            NAK -> Frame.Nak(from)
            else -> Frame.Message(to, from, cmd, body.copyOfRange(3, body.size))
        }
    }

    /**
     * True when the frame is the rig echoing our own transmission back —
     * normal on the shared bus and never a reply.
     */
    fun isEcho(frame: Frame, ourAddr: Int): Boolean =
        frame is Frame.Message && frame.from == ourAddr

    // ---- scope waveform assembly ----------------------------------------------

    /** How the rig said its scope edges are placed. */
    enum class ScopeMode { CENTER, FIXED, SCROLL_CENTER, SCROLL_FIXED }

    /** One complete scope sweep, ready for the spectrum plane. */
    class ScopeLine(
        /** 0 = main receiver, 1 = sub receiver. */
        val id: Int,
        val mode: ScopeMode,
        val lowEdgeHz: Long,
        val highEdgeHz: Long,
        /** The wanted signal left the scoped range (scroll modes). */
        val outOfRange: Boolean,
        /** Raw rig amplitude per bin, 0..levelMax. */
        val bins: ByteArray,
    ) {
        fun centerHz(): Long = lowEdgeHz + (highEdgeHz - lowEdgeHz) / 2

        fun spanHz(): Long = highEdgeHz - lowEdgeHz
    }

    private class PendingLine(
        val mode: ScopeMode,
        val lowEdgeHz: Long,
        val highEdgeHz: Long,
        val outOfRange: Boolean,
        val maxDivision: Int,
    ) {
        val bins = ArrayList<Byte>()
        var nextDivision = 2
    }

    /**
     * Reassembles the multi-frame scope waveform (command 0x27 sub 0x00).
     *
     * A sweep arrives as `maxDivision` frames: division 1 carries the header
     * (mode, edges, range flag), divisions 2.. carry the amplitude bins in
     * order. Any inconsistency — bad BCD, division out of sequence, more
     * data than the line holds — drops the sweep rather than rendering a
     * spectrum whose axis or bins are wrong.
     */
    class ScopeAssembler(private val lineLength: Int) {
        private val pending = arrayOfNulls<PendingLine>(2)

        /** Sweeps discarded for failing validation. */
        var dropped = 0L
            private set

        /**
         * Feed the data portion of one 0x27/0x00 message (everything after
         * the two sub-command bytes). Returns the finished line when this
         * frame completes a sweep.
         */
        fun push(data: ByteArray): ScopeLine? {
            if (data.size < 3) {
                dropped++
                return null
            }
            val id = data[0].toInt() and 0xFF
            if (id > 1) {
                dropped++
                return null
            }
            val division = fromBcdBe(byteArrayOf(data[1]))?.toInt() ?: run {
                dropped++
                return null
            }
            val maxDivision = fromBcdBe(byteArrayOf(data[2]))?.toInt()
            if (maxDivision == null || maxDivision < 1) {
                dropped++
                return null
            }

            val payloadStart: Int
            if (division == 1) {
                // Header frame: mode, two 5-byte BCD frequency fields, range flag.
                if (data.size < 15) {
                    pending[id] = null
                    dropped++
                    return null
                }
                val modeByte = data[3].toInt() and 0xFF
                val fa = fromBcdLe(data.copyOfRange(4, 9)) ?: run {
                    pending[id] = null
                    dropped++
                    return null
                }
                val fb = fromBcdLe(data.copyOfRange(9, 14)) ?: run {
                    pending[id] = null
                    dropped++
                    return null
                }
                val mode: ScopeMode
                val low: Long
                val high: Long
                when (modeByte) {
                    0x00 -> { mode = ScopeMode.CENTER; low = fa - fb; high = fa + fb }
                    0x01 -> { mode = ScopeMode.FIXED; low = fa; high = fb }
                    0x02 -> { mode = ScopeMode.SCROLL_CENTER; low = fa - fb; high = fa + fb }
                    0x03 -> { mode = ScopeMode.SCROLL_FIXED; low = fa; high = fb }
                    else -> {
                        pending[id] = null
                        dropped++
                        return null
                    }
                }
                if (high <= low) {
                    pending[id] = null
                    dropped++
                    return null
                }
                pending[id] = PendingLine(mode, low, high, data[14].toInt() != 0, maxDivision)
                payloadStart = 15
            } else {
                val p = pending[id]
                    // Data frame with no header seen: mid-sweep join, wait
                    // for the next sweep instead of showing a partial line.
                    ?: return null
                if (division != p.nextDivision || maxDivision != p.maxDivision) {
                    pending[id] = null
                    dropped++
                    return null
                }
                payloadStart = 3
            }

            val p = pending[id]!!
            if (p.bins.size + (data.size - payloadStart) > lineLength) {
                pending[id] = null
                dropped++
                return null
            }
            for (i in payloadStart until data.size) p.bins.add(data[i])
            p.nextDivision = division + 1

            if (division == p.maxDivision) {
                pending[id] = null
                if (p.bins.isEmpty()) {
                    dropped++
                    return null
                }
                return ScopeLine(
                    id, p.mode, p.lowEdgeHz, p.highEdgeHz, p.outOfRange, p.bins.toByteArray(),
                )
            }
            return null
        }
    }

    /**
     * Map raw scope amplitudes to dBFS-style floats for the spectrum plane:
     * [levelMax] becomes [dbMax], zero becomes [dbMin], clamped above.
     */
    fun binsToDb(bins: ByteArray, levelMax: Int, dbMin: Float, dbMax: Float): FloatArray {
        val max = maxOf(levelMax, 1).toFloat()
        val out = FloatArray(bins.size)
        for (i in bins.indices) {
            val v = minOf((bins[i].toInt() and 0xFF).toFloat(), max) / max
            out[i] = dbMin + v * (dbMax - dbMin)
        }
        return out
    }
}
