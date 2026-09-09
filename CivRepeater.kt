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
import com.isaklab.isdrproto.CatRepeaterConfig

/** Model-specific tone-selector layout proved by the corresponding CI-V guide. */
internal enum class CivToneMatrix {
    ICOM_LEGACY,
    ICOM_CROSS,
    ICOM_RECEIVE_ONLY,
}

internal data class CivRepeaterProfile(
    val caps: Int,
    val maxOffsetHz: Long,
    val offsetStepHz: Long,
    val extendedCtcss: Boolean,
    val toneMatrix: CivToneMatrix,
)

/** Complete physical register snapshot, including currently inactive presets. */
internal data class CivRepeaterState(
    val duplex: Int,
    val offsetHz: Long,
    val toneMode: Int? = null,
    val repeaterToneEnabled: Int? = null,
    val toneSquelchEnabled: Int? = null,
    val dcsEnabled: Int? = null,
    val txCtcss: Int? = null,
    val rxCtcss: Int? = null,
    val dcs: Triple<Int, Int, Int>? = null,
)

internal fun civRepeaterProfile(addr: Int, rxHz: Long): CivRepeaterProfile {
    val caps = CivModels.rigCaps(addr)
    val matrix = when (addr) {
        CivModels.ADDR_IC705, CivModels.ADDR_IC9700, CivModels.ADDR_IC905 ->
            CivToneMatrix.ICOM_CROSS
        CivModels.ADDR_ICR8600 -> CivToneMatrix.ICOM_RECEIVE_ONLY
        else -> CivToneMatrix.ICOM_LEGACY
    }
    return CivRepeaterProfile(
        caps = caps.repeaterCaps,
        maxOffsetHz = CivModels.repeaterMaxOffsetHz(addr, rxHz),
        offsetStepHz = caps.repeaterOffsetStepHz,
        extendedCtcss = caps.extendedCtcss,
        toneMatrix = matrix,
    )
}

private fun needs(caps: Int, bit: Int, what: String): String? =
    if (caps and bit != 0) null else "CAT profile does not support $what"

private fun validateToneDirection(
    kind: Int,
    polarity: Int,
    caps: Int,
    ctcssBit: Int,
    dcsBit: Int,
    direction: String,
): String? = when (kind) {
    CatRepeater.TONE_OFF -> null
    CatRepeater.TONE_CTCSS -> needs(caps, ctcssBit, "$direction CTCSS")
    CatRepeater.TONE_DCS -> needs(caps, dcsBit, "$direction DCS")
        ?: if (polarity == CatRepeater.DCS_INVERTED) {
            needs(caps, CatRepeater.CAP_DCS_POLARITY, "DCS polarity")
        } else {
            null
        }
    else -> "unknown CAT tone kind"
}

/** Validate a wire-valid request against one exact documented radio profile. */
internal fun validateCivRepeater(
    config: CatRepeaterConfig,
    profile: CivRepeaterProfile,
): String? {
    runCatching { config.encode() }.exceptionOrNull()?.let {
        return it.message ?: "invalid CAT repeater request"
    }
    if (profile.caps == 0) return "CAT profile has no documented repeater controls"

    if (config.duplex != CatRepeater.DUPLEX_SIMPLEX) {
        needs(profile.caps, CatRepeater.CAP_DUPLEX, "duplex direction")?.let { return it }
        needs(profile.caps, CatRepeater.CAP_OFFSET, "repeater offset")?.let { return it }
        if (config.offsetHz > profile.maxOffsetHz) {
            return "CAT repeater offset ${config.offsetHz} Hz exceeds this profile's " +
                "${profile.maxOffsetHz} Hz limit"
        }
        if (profile.offsetStepHz <= 0 || config.offsetHz % profile.offsetStepHz != 0L) {
            return "CAT repeater offset ${config.offsetHz} Hz is not on this profile's " +
                "${profile.offsetStepHz} Hz ladder"
        }
    }

    validateToneDirection(
        config.txKind,
        config.txPolarity,
        profile.caps,
        CatRepeater.CAP_CTCSS_TX,
        CatRepeater.CAP_DCS_TX,
        "TX",
    )?.let { return it }
    validateToneDirection(
        config.rxKind,
        config.rxPolarity,
        profile.caps,
        CatRepeater.CAP_CTCSS_RX,
        CatRepeater.CAP_DCS_RX,
        "RX",
    )?.let { return it }

    if (!profile.extendedCtcss &&
        (config.txKind == CatRepeater.TONE_CTCSS && config.txValue in intArrayOf(600, 1200) ||
            config.rxKind == CatRepeater.TONE_CTCSS && config.rxValue in intArrayOf(600, 1200))
    ) {
        return "CAT profile uses the common CTCSS ladder without 60.0/120.0 Hz"
    }

    val pair = Pair(config.txKind, config.rxKind)
    val supported = when (profile.toneMatrix) {
        CivToneMatrix.ICOM_LEGACY -> pair in setOf(
            Pair(CatRepeater.TONE_OFF, CatRepeater.TONE_OFF),
            Pair(CatRepeater.TONE_CTCSS, CatRepeater.TONE_OFF),
            Pair(CatRepeater.TONE_CTCSS, CatRepeater.TONE_CTCSS),
        )
        CivToneMatrix.ICOM_CROSS -> pair in setOf(
            Pair(CatRepeater.TONE_OFF, CatRepeater.TONE_OFF),
            Pair(CatRepeater.TONE_CTCSS, CatRepeater.TONE_OFF),
            Pair(CatRepeater.TONE_CTCSS, CatRepeater.TONE_CTCSS),
            Pair(CatRepeater.TONE_DCS, CatRepeater.TONE_DCS),
            Pair(CatRepeater.TONE_DCS, CatRepeater.TONE_OFF),
            Pair(CatRepeater.TONE_CTCSS, CatRepeater.TONE_DCS),
            Pair(CatRepeater.TONE_DCS, CatRepeater.TONE_CTCSS),
        )
        CivToneMatrix.ICOM_RECEIVE_ONLY -> pair in setOf(
            Pair(CatRepeater.TONE_OFF, CatRepeater.TONE_OFF),
            Pair(CatRepeater.TONE_OFF, CatRepeater.TONE_CTCSS),
            Pair(CatRepeater.TONE_OFF, CatRepeater.TONE_DCS),
        )
    }
    if (!supported) return "CAT profile cannot represent this TX/RX tone combination"
    if (config.txKind != CatRepeater.TONE_OFF && config.rxKind != CatRepeater.TONE_OFF &&
        config.txKind != config.rxKind
    ) {
        needs(profile.caps, CatRepeater.CAP_CROSS_TONE, "cross-tone combinations")?.let {
            return it
        }
    }
    if (pair == Pair(CatRepeater.TONE_DCS, CatRepeater.TONE_DCS) &&
        config.txValue != config.rxValue
    ) {
        return "Icom stores one shared DCS code for TX and RX"
    }
    return null
}

internal fun civCrossToneKinds(mode: Int): Pair<Int, Int>? = when (mode) {
    0x00 -> Pair(CatRepeater.TONE_OFF, CatRepeater.TONE_OFF)
    0x01 -> Pair(CatRepeater.TONE_CTCSS, CatRepeater.TONE_OFF)
    0x02, 0x09 -> Pair(CatRepeater.TONE_CTCSS, CatRepeater.TONE_CTCSS)
    0x03 -> Pair(CatRepeater.TONE_DCS, CatRepeater.TONE_DCS)
    0x06 -> Pair(CatRepeater.TONE_DCS, CatRepeater.TONE_OFF)
    0x07 -> Pair(CatRepeater.TONE_CTCSS, CatRepeater.TONE_DCS)
    0x08 -> Pair(CatRepeater.TONE_DCS, CatRepeater.TONE_CTCSS)
    else -> null
}

internal fun civCrossToneMode(config: CatRepeaterConfig): Int? =
    when (Pair(config.txKind, config.rxKind)) {
        Pair(CatRepeater.TONE_OFF, CatRepeater.TONE_OFF) -> 0x00
        Pair(CatRepeater.TONE_CTCSS, CatRepeater.TONE_OFF) -> 0x01
        Pair(CatRepeater.TONE_CTCSS, CatRepeater.TONE_CTCSS) ->
            if (config.txValue == config.rxValue) 0x02 else 0x09
        Pair(CatRepeater.TONE_DCS, CatRepeater.TONE_DCS) -> 0x03
        Pair(CatRepeater.TONE_DCS, CatRepeater.TONE_OFF) -> 0x06
        Pair(CatRepeater.TONE_CTCSS, CatRepeater.TONE_DCS) -> 0x07
        Pair(CatRepeater.TONE_DCS, CatRepeater.TONE_CTCSS) -> 0x08
        else -> null
    }
