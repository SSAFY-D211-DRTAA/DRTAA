package com.drtaa.core_model.network

data class RequestChangeRent(
    val rentId: Long,
    val rentHeadCount: Int,
    val rentDptLat: Double,
    val rentDptLon: Double,
)
