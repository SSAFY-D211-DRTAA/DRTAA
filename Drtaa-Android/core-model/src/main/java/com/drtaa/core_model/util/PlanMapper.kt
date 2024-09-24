package com.drtaa.core_model.util

import java.time.LocalDate
import java.time.format.DateTimeFormatter
import java.time.format.TextStyle
import java.util.Locale

/**
 * MM.dd E 형식으로 날짜 포맷
 */
fun String.toDate(): String {
    val date = LocalDate.parse(this, DateTimeFormatter.ISO_DATE)

    val monthDayFormatter = DateTimeFormatter.ofPattern("MM.dd")
    val dayOfWeek = date.dayOfWeek.getDisplayName(TextStyle.SHORT, Locale.KOREAN)

    return "${date.format(monthDayFormatter)} $dayOfWeek"
}