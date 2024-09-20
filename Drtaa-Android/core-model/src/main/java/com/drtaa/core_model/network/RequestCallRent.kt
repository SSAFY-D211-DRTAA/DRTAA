package com.drtaa.core_model.network

data class RequestCallRent(
    val rentHeadCount: Int,
    val rentTime: Int,
    val rentPrice: Long,
    val rentStartTime: String,
    val rentEndTime: String,
    val rentDptLat: Double,
    val rentDptLon: Double
)
