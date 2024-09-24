package com.drtaa.core_model.network

data class ResponseDrivingCar(
    val rentCarId: Int,
    val rentCarDrivingStatus: String,
    val rentCarNumber: String? = null,
)
