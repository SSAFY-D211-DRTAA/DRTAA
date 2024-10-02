package com.drtaa.core_model.rent

data class CarPosition(
    val latitude: Double,
    val longitude: Double,
    val rentId: Int,
    val travelId: Int,
    val travelDatesId: Int,
    val datePlacesId: Int
)
