package com.drtaa.core_model.rent

import java.util.Locale

data class Time(
    val hour: Int,
    val minute: Int
) {
    override fun toString(): String {
        return String.format(Locale.ROOT, "%02d:%02d", hour, minute)
    }
}
