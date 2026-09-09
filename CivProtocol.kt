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
    const val CMD_READ_REPEATER_OFFSET = 0x0C
    const val CMD_SET_REPEATER_OFFSET = 0x0D
    const val CMD_DUPLEX = 0x0F
    const val CMD_ATTENUATOR = 0x11
    const val CMD_LEVEL = 0x14
    const val CMD_READ_METER = 0x15
    const val CMD_FUNC = 0x16
    const val CMD_READ_ID = 0x19
    const val CMD_MEM = 0x1A
    const val CMD_TONE = 0x1B
    const val CMD_PTT = 0x1C
    const val CMD_MODE_DATA = 0x26
    const val CMD_SCOPE = 0x27

    const val SUB_LEVEL_AF = 0x01
    const val SUB_LEVEL_RF = 0x02
    const val SUB_LEVEL_SQL = 0x03
    const val SUB_LEVEL_NR = 0x06
    const val SUB_LEVEL_PBT_IN = 0x07
    const val SUB_LEVEL_PBT_OUT = 0x08
    const val SUB_LEVEL_RFPOWER = 0x0A

    const val SUB_FUNC_PREAMP = 0x02
    const val SUB_FUNC_AGC = 0x12
    const val SUB_FUNC_NB = 0x22
    const val SUB_FUNC_NR = 0x40
    const val SUB_FUNC_NOTCH_AUTO = 0x41
    const val SUB_FUNC_REPEATER_TONE = 0x42
    const val SUB_FUNC_TONE_SQUELCH = 0x43
    const val SUB_FUNC_DCS = 0x4B
    const val SUB_FUNC_TONE_MODE = 0x5D

    const val SUB_TONE_TX = 0x00
    const val SUB_TONE_RX = 0x01
    const val SUB_TONE_DCS = 0x02

    const val SUB_MEM_FILTER_WIDTH = 0x03

    /**
     * AGC time-constant presets (cmd 0x16 sub 0x12). The current rig line
     * has no AGC-off code on this command — off is a per-mode time-constant
     * of 0, a different setting entirely.
     */
    const val AGC_FAST = 0x01
    const val AGC_MID = 0x02
    const val AGC_SLOW = 0x03

    const val SUB_METER_S = 0x02
    const val SUB_METER_POWER = 0x11
    const val SUB_METER_SWR = 0x12
    const val SUB_METER_ALC = 0x13
    const val SUB_PTT = 0x00
    const val SUB_ID = 0x00
    const val SUB_MODE_DATA_SELECTED = 0x00

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

    /** Highest frequency expressible in the 12-digit CI-V BCD field. */
    const val MAX_FREQ_HZ = 99_999_999_999L

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

    /** Decode an exact 5-byte normal-band or 6-byte IC-905 frequency. */
    fun parseFrequency(bytes: ByteArray): Long? {
        if (bytes.size != 5 && bytes.size != 6) return null
        val hz = fromBcdLe(bytes) ?: return null
        return hz.takeIf { it <= MAX_FREQ_HZ }
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

    /** Pack [value] as packed big-endian BCD, two digits per byte. */
    fun toBcdBe(value: Long, len: Int): ByteArray? {
        if (value < 0) return null
        var v = value
        val out = ByteArray(len)
        for (i in out.indices.reversed()) {
            val lo = (v % 10).toInt()
            v /= 10
            val hi = (v % 10).toInt()
            v /= 10
            out[i] = ((hi shl 4) or lo).toByte()
        }
        return out.takeIf { v == 0L }
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

    /** Null beyond 12 digits; the sixth byte is used only above 9.999 GHz. */
    fun writeFrequency(to: Int, hz: Long): ByteArray? {
        if (hz !in 0..MAX_FREQ_HZ) return null
        val bcd = toBcdLe(hz, if (hz <= 9_999_999_999L) 5 else 6) ?: return null
        return buildFrame(to, CONTROLLER_ADDR, byteArrayOf(CMD_WRITE_FREQ.toByte()) + bcd)
    }

    fun readMode(to: Int): ByteArray =
        buildFrame(to, CONTROLLER_ADDR, byteArrayOf(CMD_READ_MODE.toByte()))!!

    /**
     * Physical mode state. [filter] is null when a legacy 0x04 reply omits the
     * optional filter byte; callers must not turn that absence into a FIL
     * confirmation.
     */
    data class ModeState(val mode: Int, val data: Boolean, val filter: Int?)

    /** Read the selected VFO mode, DATA flag and IF filter. */
    fun readModeData(to: Int): ByteArray = buildFrame(
        to,
        CONTROLLER_ADDR,
        byteArrayOf(CMD_MODE_DATA.toByte(), SUB_MODE_DATA_SELECTED.toByte()),
    )!!

    /** [filter] is the rig's passband selection 1..3. */
    fun writeMode(to: Int, mode: Int, filter: Int): ByteArray? {
        if (mode !in 0..0x23 || filter !in 1..3) return null
        return buildFrame(
            to, CONTROLLER_ADDR,
            byteArrayOf(CMD_WRITE_MODE.toByte(), mode.toByte(), filter.toByte()),
        )
    }

    /** Selected-VFO mode with the explicit DATA flag (0x26 sub 0x00). */
    fun writeModeData(to: Int, mode: Int, data: Boolean, filter: Int): ByteArray? {
        if (mode !in 0..0x23 || filter !in 1..3) return null
        return buildFrame(
            to,
            CONTROLLER_ADDR,
            byteArrayOf(
                CMD_MODE_DATA.toByte(),
                SUB_MODE_DATA_SELECTED.toByte(),
                mode.toByte(),
                if (data) 1 else 0,
                filter.toByte(),
            ),
        )
    }

    fun parseMode(data: ByteArray): ModeState? {
        if (data.size !in 1..2) return null
        val mode = data[0].toInt() and 0xFF
        val filter = data.getOrNull(1)?.toInt()?.and(0xFF)
        if (mode > 0x23 || filter != null && filter !in 1..3) return null
        return ModeState(mode, false, filter)
    }

    fun parseModeData(data: ByteArray): ModeState? {
        if (data.size != 4 ||
            (data[0].toInt() and 0xFF) != SUB_MODE_DATA_SELECTED
        ) return null
        val mode = data[1].toInt() and 0xFF
        val dataFlag = data[2].toInt() and 0xFF
        val filter = data[3].toInt() and 0xFF
        if (mode > 0x23 || dataFlag !in 0..1 || filter !in 1..3) return null
        return ModeState(mode, dataFlag == 1, filter)
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

    /** Set simplex, duplex-minus, or duplex-plus (`0x0F 10/11/12`). */
    fun setDuplex(to: Int, duplex: Int): ByteArray? {
        val code = when (duplex) {
            com.isaklab.isdrproto.CatRepeater.DUPLEX_SIMPLEX -> 0x10
            com.isaklab.isdrproto.CatRepeater.DUPLEX_MINUS -> 0x11
            com.isaklab.isdrproto.CatRepeater.DUPLEX_PLUS -> 0x12
            else -> return null
        }
        return buildFrame(to, CONTROLLER_ADDR, byteArrayOf(CMD_DUPLEX.toByte(), code.toByte()))
    }

    fun readDuplex(to: Int): ByteArray =
        buildFrame(to, CONTROLLER_ADDR, byteArrayOf(CMD_DUPLEX.toByte()))!!

    fun parseDuplex(data: ByteArray): Int? = when {
        data.contentEquals(byteArrayOf(0x10)) -> com.isaklab.isdrproto.CatRepeater.DUPLEX_SIMPLEX
        data.contentEquals(byteArrayOf(0x11)) -> com.isaklab.isdrproto.CatRepeater.DUPLEX_MINUS
        data.contentEquals(byteArrayOf(0x12)) -> com.isaklab.isdrproto.CatRepeater.DUPLEX_PLUS
        else -> null
    }

    /** CI-V offsets are unsigned little-endian BCD in 100 Hz units. */
    fun setRepeaterOffset(to: Int, offsetHz: Long, bytes: Int): ByteArray? {
        if (offsetHz < 0 || offsetHz % 100L != 0L || bytes !in 3..4) return null
        val bcd = toBcdLe(offsetHz / 100L, bytes) ?: return null
        return buildFrame(
            to,
            CONTROLLER_ADDR,
            byteArrayOf(CMD_SET_REPEATER_OFFSET.toByte()) + bcd,
        )
    }

    fun readRepeaterOffset(to: Int): ByteArray =
        buildFrame(to, CONTROLLER_ADDR, byteArrayOf(CMD_READ_REPEATER_OFFSET.toByte()))!!

    fun parseRepeaterOffset(data: ByteArray, bytes: Int): Long? {
        if (data.size != bytes || bytes !in 3..4) return null
        return fromBcdLe(data)?.let { runCatching { Math.multiplyExact(it, 100L) }.getOrNull() }
    }

    /** Repeater/TSQL tone frequency, three-byte big-endian BCD in tenths Hz. */
    fun setCtcssTone(to: Int, sub: Int, tenthsHz: Int): ByteArray? {
        if (sub != SUB_TONE_TX && sub != SUB_TONE_RX || tenthsHz < 0) return null
        val bcd = toBcdBe(tenthsHz.toLong(), 3) ?: return null
        return buildFrame(to, CONTROLLER_ADDR, byteArrayOf(CMD_TONE.toByte(), sub.toByte()) + bcd)
    }

    fun readTone(to: Int, sub: Int): ByteArray? {
        if (sub != SUB_TONE_TX && sub != SUB_TONE_RX && sub != SUB_TONE_DCS) return null
        return buildFrame(to, CONTROLLER_ADDR, byteArrayOf(CMD_TONE.toByte(), sub.toByte()))
    }

    fun parseCtcssTone(data: ByteArray, sub: Int): Int? {
        if (data.size != 4 || (data[0].toInt() and 0xFF) != sub ||
            sub != SUB_TONE_TX && sub != SUB_TONE_RX
        ) return null
        return fromBcdBe(data.copyOfRange(1, data.size))?.toInt()
    }

    /** DCS code plus independent TX/RX polarity. */
    fun setDcs(to: Int, code: Int, txPolarity: Int, rxPolarity: Int): ByteArray? {
        if (code < 0 || txPolarity !in 0..1 || rxPolarity !in 0..1) return null
        val bcd = toBcdBe2(code) ?: return null
        return buildFrame(
            to,
            CONTROLLER_ADDR,
            byteArrayOf(
                CMD_TONE.toByte(), SUB_TONE_DCS.toByte(),
                ((txPolarity shl 4) or rxPolarity).toByte(), bcd[0], bcd[1],
            ),
        )
    }

    fun parseDcs(data: ByteArray): Triple<Int, Int, Int>? {
        if (data.size != 4 || (data[0].toInt() and 0xFF) != SUB_TONE_DCS) return null
        val polarity = data[1].toInt() and 0xFF
        val tx = polarity shr 4
        val rx = polarity and 0x0F
        if (tx !in 0..1 || rx !in 0..1) return null
        val code = fromBcdBe(data.copyOfRange(2, data.size))?.toInt() ?: return null
        return Triple(code, tx, rx)
    }

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

    /** Parse one exact level read-back: subcommand plus 0000..0255 BCD. */
    fun parseLevel(data: ByteArray, sub: Int): Int? {
        if (data.size != 3 || (data[0].toInt() and 0xFF) != sub) return null
        val value = fromBcdBe(data.copyOfRange(1, data.size)) ?: return null
        return value.toInt().takeIf { value in 0L..255L }
    }

    /**
     * Function setting (cmd 0x16): on/off toggles and small enumerations
     * travel as a single plain data byte, not BCD-scaled levels.
     */
    fun setFunc(to: Int, sub: Int, data: Int): ByteArray =
        buildFrame(
            to, CONTROLLER_ADDR,
            byteArrayOf(CMD_FUNC.toByte(), sub.toByte(), data.toByte()),
        )!!

    fun readFunc(to: Int, sub: Int): ByteArray =
        buildFrame(to, CONTROLLER_ADDR, byteArrayOf(CMD_FUNC.toByte(), sub.toByte()))!!

    /**
     * Attenuator (cmd 0x11): the dB amount itself is the single data byte,
     * packed as two BCD digits (12 dB = 0x12); 0 switches the attenuator
     * off. Null beyond the two-digit field.
     */
    fun setAttenuator(to: Int, db: Int): ByteArray? {
        if (db !in 0..99) return null
        val bcd = ((db / 10) shl 4) or (db % 10)
        return buildFrame(
            to, CONTROLLER_ADDR,
            byteArrayOf(CMD_ATTENUATOR.toByte(), bcd.toByte()),
        )
    }

    /**
     * Width-code table index for [hz] in the given operating mode, snapped
     * to the nearest code the rig accepts.
     *
     * SSB/CW/RTTY share one table: codes 0..9 are 50..500 Hz in 50 Hz
     * steps, 10..40 are 600..3600 Hz in 100 Hz steps. AM has its own:
     * codes 0..49 are 200..10000 Hz in 200 Hz steps. FM (and anything
     * else) has no width command — null.
     */
    fun widthCodeForMode(mode: Int, hz: Int): Int? {
        val codeToHz: (Int) -> Int
        val maxCode: Int
        when (mode) {
            MODE_LSB, MODE_USB, MODE_CW, MODE_CW_R, MODE_RTTY, MODE_RTTY_R -> {
                codeToHz = { c -> if (c <= 9) 50 * (c + 1) else 600 + 100 * (c - 10) }
                maxCode = 40
            }
            MODE_AM -> {
                codeToHz = { c -> 200 * (c + 1) }
                maxCode = 49
            }
            else -> return null
        }
        return (0..maxCode).minByOrNull { kotlin.math.abs(codeToHz(it) - hz) }
    }

    /**
     * IF filter width (cmd 0x1A sub 0x03). The data byte is the width CODE
     * from the per-mode table, itself packed as two BCD digits (code 40 =
     * 0x40). Null when the mode has no width command.
     *
     * 0x1A subcommands are model-family specific — on other families this
     * address means something unrelated — so the caller gates this on a rig
     * known to speak it.
     */
    fun setFilterWidth(to: Int, mode: Int, hz: Int): ByteArray? {
        val code = widthCodeForMode(mode, hz) ?: return null
        val bcd = ((code / 10) shl 4) or (code % 10)
        return buildFrame(
            to, CONTROLLER_ADDR,
            byteArrayOf(CMD_MEM.toByte(), SUB_MEM_FILTER_WIDTH.toByte(), bcd.toByte()),
        )
    }

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
