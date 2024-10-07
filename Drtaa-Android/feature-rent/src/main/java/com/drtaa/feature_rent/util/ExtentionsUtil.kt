package com.drtaa.feature_rent.util

import com.prolificinteractive.materialcalendarview.CalendarDay
import org.threeten.bp.format.DateTimeFormatter

fun CalendarDay?.formatToYearMonthDay(): String {
    return this?.date?.format(DateTimeFormatter.ofPattern("yyyy.MM.dd (E)")) ?: ""
}