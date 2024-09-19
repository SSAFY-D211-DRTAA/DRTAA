package com.drtaa.core_model.network

import com.drtaa.core_model.rent.RentCar

data class ResponseRentCar(
    val available: Boolean,
    val rentCar: RentCar
)