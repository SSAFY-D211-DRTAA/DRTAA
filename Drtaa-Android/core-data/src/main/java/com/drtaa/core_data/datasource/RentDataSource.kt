package com.drtaa.core_data.datasource

import com.drtaa.core_model.network.ResponseRentCar

interface RentDataSource {
    suspend fun getUnassignedCar(): ResponseRentCar
}