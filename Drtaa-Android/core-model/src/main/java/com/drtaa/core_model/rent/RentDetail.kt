package com.drtaa.core_model.rent

import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
import java.util.Locale

data class RentDetail(
    val rentId: Long? = null,
    val rentCarId: Int,
    val rentCarManufacturer: String,
    val rentCarModel: String,
    val rentCarNumber: String,
    val rentCarScheduleId: Int,
    val rentCreatedAt: String,
    val rentDptLat: Double,
    val rentDptLon: Double,
    val rentEndTime: String,
    val rentHeadCount: Int,
    val rentCarImg: String? = null,
    val rentPrice: Int,
    val rentStartTime: String,
    val rentStatus: String,
    val rentTime: Double,
    val travelId: Long,
    val rentStartLocation: String = ""
) {
    fun rentScheduleToString(dateString: String): String {
        val dateTime = LocalDateTime.parse(dateString)
        val outputFormatter = DateTimeFormatter.ofPattern("MM.dd(E) HH:mm", Locale("ko", "KR"))

        return dateTime.format(outputFormatter)
    }
}