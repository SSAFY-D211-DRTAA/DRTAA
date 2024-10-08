package com.drtaa.core_model.network

data class RequestRentCarCall(
    val rentId: Long,
    val userLat: Double,
    val userLon: Double,
    val travelId: Long,
    val travelDatesId: Long,
    val datePlacesId: Long,
)
