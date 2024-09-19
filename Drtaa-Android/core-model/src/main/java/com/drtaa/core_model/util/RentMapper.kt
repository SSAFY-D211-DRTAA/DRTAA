package com.drtaa.core_model.util

import com.drtaa.core_model.rent.RentSchedule
import java.time.LocalDateTime

/**
 * RentSchedule을 LocalDateTime으로 변환하는 함수
 */
fun RentSchedule.toLocalDateTime(): LocalDateTime {
    return LocalDateTime.of(
        this.year,
        this.month,
        this.date,
        this.hour,
        this.minute
    )
}