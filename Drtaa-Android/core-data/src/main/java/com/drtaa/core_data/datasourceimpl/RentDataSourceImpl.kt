package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.RentDataSource
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.rent.RentCar
import com.drtaa.core_network.api.RentAPI
import javax.inject.Inject

class RentDataSourceImpl @Inject constructor(
    private val rentAPI: RentAPI
) : RentDataSource {
    override suspend fun getUnassignedCar(rentSchedule: RequestUnassignedCar): RentCar {
        return rentAPI.getUnassignedCar(rentSchedule)
    }
}