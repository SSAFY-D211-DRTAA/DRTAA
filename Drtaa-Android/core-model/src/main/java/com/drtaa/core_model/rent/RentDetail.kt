package com.drtaa.core_model.rent

data class RentDetail(
    val rentId: Long,
    val rentCarId: Int,
    val rentCarManufacturer: String,
    val rentCarModel: String,
    val rentCarNumber: String,
    val rentCarScheduleId: Int,
    val rentCreatedAt: String,
    val rentDptLat: Int,
    val rentDptLon: Int,
    val rentEndTime: String,
    val rentHeadCount: Int,
    val rentPrice: Int,
    val rentStartTime: String,
    val rentStatus: String,
    val rentTime: Int
)