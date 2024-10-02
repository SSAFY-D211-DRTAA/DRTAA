package com.drtaa.core_model.rent

data class CarPosition(
    val latitude: Double,
    val longitude: Double,
    val rentId: Long,
    val travelId: Long,
    val travelDatesId: Long,
    val datePlacesId: Long
)
