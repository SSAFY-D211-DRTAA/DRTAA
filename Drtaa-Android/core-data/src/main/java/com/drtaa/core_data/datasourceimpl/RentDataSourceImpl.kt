package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.RentDataSource
import com.drtaa.core_model.network.ResponseRentCar
import com.drtaa.core_network.api.RentAPI
import javax.inject.Inject

class RentDataSourceImpl @Inject constructor(
    private val rentAPI: RentAPI
) : RentDataSource {
    override suspend fun getUnassignedCar(): ResponseRentCar {
        return rentAPI.getUnassignedCar()
    }
}