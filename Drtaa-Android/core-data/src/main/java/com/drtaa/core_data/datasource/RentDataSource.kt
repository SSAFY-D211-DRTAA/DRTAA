package com.drtaa.core_data.datasource

import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.network.ResponseCallRent
import com.drtaa.core_model.rent.RentCar

interface RentDataSource {
    suspend fun getUnassignedCar(rentSchedule: RequestUnassignedCar): RentCar
    suspend fun callRent(requestCallRent: RequestCallRent): ResponseCallRent
}