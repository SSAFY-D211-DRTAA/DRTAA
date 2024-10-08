package com.drtaa.core_model.network

data class ResponseRentCarCall(
    val rentCarId: Long,
    val rentCarLat: Double,
    val rentCarLon: Double,
    val rentId: Long,
    val travelId: Long,
    val travelDatesId: Long,
    val datePlacesId: Long,
)
