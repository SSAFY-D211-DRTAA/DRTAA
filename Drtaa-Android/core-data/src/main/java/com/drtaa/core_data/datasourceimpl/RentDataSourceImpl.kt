package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.RentDataSource
import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.RequestDrivingCar
import com.drtaa.core_model.network.RequestRentCarCall
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.network.ResponseDrivingCar
import com.drtaa.core_model.network.ResponseRentCarCall
import com.drtaa.core_model.network.ResponseRentStateAll
import com.drtaa.core_model.rent.RentCar
import com.drtaa.core_model.rent.RentDetail
import com.drtaa.core_model.rent.RentSimple
import com.drtaa.core_network.api.RentAPI
import com.drtaa.core_network.api.RentCarAPI
import javax.inject.Inject

class RentDataSourceImpl @Inject constructor(
    private val rentAPI: RentAPI,
    private val rentCarAPI: RentCarAPI,
) : RentDataSource {
    override suspend fun getUnassignedCar(rentSchedule: RequestUnassignedCar): RentCar {
        return rentCarAPI.getUnassignedCar(rentSchedule)
    }

    override suspend fun callAssignedCar(requestCallCar: RequestRentCarCall): ResponseRentCarCall {
        return rentCarAPI.callAssignedCar(requestCallCar)
    }

    override suspend fun callFirstAssignedCar(rentId: Long): ResponseRentCarCall {
        return rentCarAPI.callFirstAssignedCar(rentId)
    }

    override suspend fun editDriveStatus(request: RequestDrivingCar): ResponseDrivingCar {
        return rentCarAPI.editDriveStatus(request)
    }

    override suspend fun getDriveStatus(rentCarId: Long): ResponseDrivingCar {
        return rentCarAPI.getDriveStatus(rentCarId)
    }

    override suspend fun callRent(requestCallRent: RequestCallRent): RentDetail {
        return rentAPI.callAllRent(requestCallRent)
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

    override suspend fun getAllRentState(): List<ResponseRentStateAll> {
        return rentAPI.getAllRentState()
    }
}