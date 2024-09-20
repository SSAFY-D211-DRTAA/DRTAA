package com.drtaa.core_model.rent

data class RentCar(
    val available: Boolean,
    val rentCarDrivingStatus: String,
    val rentCarId: Int,
    val rentCarManufacturer: String,
    val rentCarModel: String,
    val rentCarNumber: String,
    val rentCarImg: String
)