package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.RentDataSource
import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestChangeRent
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.RequestDuplicatedSchedule
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.rent.RentCar
import com.drtaa.core_model.network.RequestRentExtend
import com.drtaa.core_model.network.ResponseRentStateAll
import com.drtaa.core_model.rent.RentDetail
import com.drtaa.core_model.rent.RentSimple
import com.drtaa.core_network.api.RentAPI
import javax.inject.Inject

class RentDataSourceImpl @Inject constructor(
    private val rentAPI: RentAPI,
) : RentDataSource {

    override suspend fun callRent(requestCallRent: RequestCallRent): RentDetail {
        return rentAPI.callRent(requestCallRent)
    }

    override suspend fun changeRent(requestChangeRent: RequestChangeRent): String {
        return rentAPI.changeRent(requestChangeRent)
    }

    override suspend fun extendRentTime(requestRentExtend: RequestRentExtend): String {
        return rentAPI.extendRentTime(requestRentExtend)
    }

    override suspend fun getAllCompletedRent(rentId: Long): List<ResponseRentStateAll> {
        return rentAPI.getAllCompletedRent(rentId)
    }

    override suspend fun completeRent(requestCompleteRent: RequestCompleteRent): String {
        return rentAPI.completeRent(requestCompleteRent)
    }

    override suspend fun cancelRent(requestCompleteRent: RequestCompleteRent): String {
        return rentAPI.cancelRent(requestCompleteRent)
    }

    override suspend fun getRentDetail(rentId: Long): RentDetail {
        return rentAPI.getRentDetail(rentId)
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

    override suspend fun getAllRentState(): List<ResponseRentStateAll> {
        return rentAPI.getAllRentState()
    }
}