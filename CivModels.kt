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

import com.isaklab.isdrproto.CatRepeater

/**
 * What each CI-V address is known to be, and what its scope produces.
 *
 * The protocol is uniform across the line; the address and the scope
 * geometry are the only per-model facts the driver needs. An address not
 * listed here still gets full rig control — it just reports no spectrum
 * capability until a waveform frame proves otherwise.
 */
object CivModels {

    val COMMON_SCOPE_SPANS_HZ = listOf(
        5_000L, 10_000L, 20_000L, 50_000L, 100_000L, 200_000L, 500_000L, 1_000_000L,
    )

    val R8600_SCOPE_SPANS_HZ = COMMON_SCOPE_SPANS_HZ + listOf(2_000_000L, 5_000_000L)

    val IC905_SCOPE_SPANS_HZ = COMMON_SCOPE_SPANS_HZ +
        listOf(2_000_000L, 5_000_000L, 10_000_000L, 20_000_000L, 50_000_000L)

    /** Geometry and level scale of a rig's scope waveform. */
    data class ScopeCaps(
        /** Amplitude bins in one complete sweep. */
        val lineLength: Int,
        /** Raw bin value that represents the top of the scale. */
        val levelMax: Int,
        /** Bottom of the scale on the display, in dB. */
        val dbMin: Float,
        /** Top of the scale on the display, in dB. */
        val dbMax: Float,
        /** Exact full displayed widths; CI-V 0x27/0x15 carries half this value. */
        val spansHz: List<Long>,
    ) {
        companion object {
            /**
             * The 475-bin family uses amplitude 0..160 over its 80 dB
             * display window. IC-7610 has its own geometry below.
             */
            fun standard(spansHz: List<Long> = COMMON_SCOPE_SPANS_HZ) = ScopeCaps(
                lineLength = 475,
                levelMax = 160,
                dbMin = -80f,
                dbMax = 0f,
                spansHz = spansHz,
            )
        }
    }

    /** Exact model facts needed before an RF-affecting repeater command is legal. */
    data class RigCaps(
        val hasTx: Boolean,
        val repeaterCaps: Int,
        val repeaterOffsetBytes: Int,
        val repeaterOffsetStepHz: Long,
        val extendedCtcss: Boolean,
    )

    /** Default bus addresses of the scope-capable rigs. */
    const val ADDR_IC7300 = 0x94
    const val ADDR_IC7610 = 0x98
    const val ADDR_IC9700 = 0xA2
    const val ADDR_IC705 = 0xA4
    const val ADDR_IC905 = 0xAC
    const val ADDR_ICR8600 = 0x96
    const val ADDR_IC7851 = 0x8E

    /** Scope geometry for a bus address; null for control-only rigs. */
    fun scopeCaps(addr: Int): ScopeCaps? = when (addr) {
        ADDR_IC7300, ADDR_IC9700, ADDR_IC705 -> ScopeCaps.standard()
        ADDR_IC7610 -> ScopeCaps(689, 200, -100f, 0f, COMMON_SCOPE_SPANS_HZ)
        ADDR_IC905 -> ScopeCaps.standard(IC905_SCOPE_SPANS_HZ)
        ADDR_ICR8600 -> ScopeCaps.standard(R8600_SCOPE_SPANS_HZ)
        else -> null
    }

    /** Models whose selected-VFO 0x26 command proves the DATA flag. */
    fun supportsModeData(addr: Int): Boolean = addr == ADDR_IC7300 ||
        addr == ADDR_IC7610 || addr == ADDR_IC9700 || addr == ADDR_IC705 ||
        addr == ADDR_IC905 || addr == ADDR_IC7851

    /**
     * Repeater capability is address-specific. Generic CI-V syntax is not
     * evidence that an unknown radio implements any of these registers.
     */
    fun rigCaps(addr: Int): RigCaps {
        val ctcss = CatRepeater.CAP_CTCSS_TX or CatRepeater.CAP_CTCSS_RX
        val full = CatRepeater.CAP_DUPLEX or CatRepeater.CAP_OFFSET or ctcss or
            CatRepeater.CAP_DCS_TX or CatRepeater.CAP_DCS_RX or
            CatRepeater.CAP_DCS_POLARITY or CatRepeater.CAP_CROSS_TONE
        return when (addr) {
            ADDR_IC7300 -> RigCaps(true, ctcss, 0, 0, false)
            ADDR_IC705 -> RigCaps(true, full, 3, 100, true)
            ADDR_IC7610 -> RigCaps(true, ctcss, 0, 0, false)
            ADDR_IC9700 -> RigCaps(true, full, 3, 100, true)
            ADDR_IC905 -> RigCaps(true, full, 3, 100, true)
            ADDR_IC7851 -> RigCaps(true, ctcss, 0, 0, false)
            ADDR_ICR8600 -> RigCaps(
                hasTx = false,
                repeaterCaps = CatRepeater.CAP_DUPLEX or CatRepeater.CAP_OFFSET or
                    CatRepeater.CAP_CTCSS_RX or CatRepeater.CAP_DCS_RX or
                    CatRepeater.CAP_DCS_POLARITY,
                repeaterOffsetBytes = 4,
                repeaterOffsetStepHz = 1_000,
                extendedCtcss = false,
            )
            else -> RigCaps(true, 0, 0, 0, false)
        }
    }

    fun supportsMode(addr: Int, mode: Int, data: Boolean): Boolean {
        if (data && mode !in listOf(0, 1, 2, 5)) return false
        return when (mode) {
            in 0..5, 7, 8 -> true
            6 -> !data && addr in listOf(ADDR_IC705, ADDR_ICR8600)
            0x12, 0x13 -> !data && addr in listOf(ADDR_IC7610, ADDR_IC7851)
            else -> false
        }
    }

    fun preampMax(addr: Int, rxHz: Long): Int = when (addr) {
        ADDR_IC9700 -> 3
        ADDR_IC905, ADDR_ICR8600 -> 1
        ADDR_IC705 -> if (rxHz in 144_000_000L until 148_000_000L || rxHz in 430_000_000L until 450_000_000L) 1 else 2
        else -> 2
    }

    fun attenuatorAllowed(addr: Int, rxHz: Long, db: Int): Boolean = when (addr) {
        ADDR_IC7300 -> db == 0 || db == 20
        ADDR_IC705 -> db == 0 || db == 20 &&
            rxHz !in 144_000_000L until 148_000_000L && rxHz !in 430_000_000L until 450_000_000L
        ADDR_IC7610 -> db in 0..45 && db % 3 == 0
        ADDR_IC7851 -> db in 0..21 && db % 3 == 0
        ADDR_IC9700 -> db == 0 || db == 10
        ADDR_IC905 -> db == 0 || db == 10 && rxHz < 1_300_000_000L
        ADDR_ICR8600 -> db in 0..30 && db % 10 == 0
        else -> db in 0..45
    }

    private fun interpolate(raw: Int, points: List<Pair<Int, Double>>): Double {
        if (raw <= points.first().first) return points.first().second
        for ((a, b) in points.zipWithNext()) if (raw <= b.first) {
            return a.second + (raw - a.first).toDouble() / (b.first - a.first) * (b.second - a.second)
        }
        return points.last().second
    }

    /** Display-meter interpolation between published CI-V anchors; not RF calibration. */
    fun smeterDbm(raw: Int): Double = interpolate(raw, listOf(0 to -127.0, 120 to -73.0, 241 to -13.0))
    fun powerFraction(addr: Int, raw: Int): Double = interpolate(raw, listOf(
        0 to 0.0, 143 to 0.5, (if (addr == ADDR_IC7610) 212 else 213) to 1.0, 255 to 1.2,
    ))
    fun swr(raw: Int): Double = interpolate(raw, listOf(0 to 1.0, 48 to 1.5, 80 to 2.0, 120 to 3.0, 240 to 6.0))

    /** Native offset maximum at the current receive frequency. */
    fun repeaterMaxOffsetHz(addr: Int, rxHz: Long): Long = when (addr) {
        ADDR_IC705 -> 9_999_900
        ADDR_IC9700, ADDR_IC905 -> if (rxHz >= 1_200_000_000L) 99_999_900 else 9_999_900
        ADDR_ICR8600 -> 299_999_000
        else -> 0
    }

    /** Human name for a bus address, when the default assignment is known. */
    fun modelName(addr: Int): String? = when (addr) {
        ADDR_IC7300 -> "IC-7300"
        ADDR_IC7610 -> "IC-7610"
        ADDR_IC9700 -> "IC-9700"
        ADDR_IC705 -> "IC-705"
        ADDR_IC905 -> "IC-905"
        ADDR_ICR8600 -> "IC-R8600"
        ADDR_IC7851 -> "IC-7851"
        0x88 -> "IC-7100"
        0x8C -> "IC-7200"
        0x76 -> "IC-7400"
        0xA0 -> "IC-7410"
        0x70 -> "IC-7000"
        0x5E -> "IC-718"
        else -> null
    }
}
