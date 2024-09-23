package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.RentDataSource
import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.RequestDuplicatedSchedule
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.rent.RentCar
import com.drtaa.core_model.rent.RentDetail
import com.drtaa.core_model.rent.RentSimple
import com.drtaa.core_network.api.RentAPI
import javax.inject.Inject

class RentDataSourceImpl @Inject constructor(
    private val rentAPI: RentAPI
) : RentDataSource {
    override suspend fun getUnassignedCar(rentSchedule: RequestUnassignedCar): RentCar {
        return rentAPI.getUnassignedCar(rentSchedule)
    }

    override suspend fun callRent(requestCallRent: RequestCallRent): RentDetail {
        return rentAPI.callRent(requestCallRent)
    }

    override suspend fun completeRent(requestCompleteRent: RequestCompleteRent) {
        rentAPI.completeRent(requestCompleteRent)
    }

    override suspend fun getRentHistory(): List<RentSimple> {
        return rentAPI.getRentHistory()
    }

    override suspend fun getCurrentRent(): RentDetail {
        return rentAPI.getCurrentRent()
    }

    override suspend fun checkDuplicatedRent(rentSchedule: RequestDuplicatedSchedule): Boolean {
        return rentAPI.checkDuplicatedRent(rentSchedule)
    }
}