package com.drtaa.core_model.rent

data class RentCar(
    val rentCarDrivingStatus: String,
    val rentCarId: Int,
    val rentCarIsDispatch: Boolean,
    val rentCarManufacturer: String,
    val rentCarModel: String,
    val rentCarNumber: String,
    val rentCarImg: String
)