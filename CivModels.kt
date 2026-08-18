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
 * What each CI-V address is known to be, and what its scope produces.
 *
 * The protocol is uniform across the line; the address and the scope
 * geometry are the only per-model facts the driver needs. An address not
 * listed here still gets full rig control — it just reports no spectrum
 * capability until a waveform frame proves otherwise.
 */
object CivModels {

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
    ) {
        companion object {
            /**
             * The waveform format is identical across every scope-capable
             * rig to date: 475 bins, amplitude 0..160 spanning an 80 dB
             * window.
             */
            fun standard() = ScopeCaps(lineLength = 475, levelMax = 160, dbMin = -80f, dbMax = 0f)
        }
    }

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
        ADDR_IC7300, ADDR_IC7610, ADDR_IC9700, ADDR_IC705, ADDR_IC905, ADDR_ICR8600,
        ADDR_IC7851,
        -> ScopeCaps.standard()
        else -> null
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
