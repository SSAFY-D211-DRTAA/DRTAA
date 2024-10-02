package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.RentCarDataSource
import com.drtaa.core_model.network.RequestCarStatus
import com.drtaa.core_model.network.RequestDrivingCar
import com.drtaa.core_model.network.RequestRentCarCall
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.network.ResponseDrivingCar
import com.drtaa.core_model.network.ResponseRentCarCall
import com.drtaa.core_model.rent.RentCar
import com.drtaa.core_network.api.RentCarAPI
import javax.inject.Inject

class RentCarDataSourceImpl @Inject constructor(
    private val rentCarAPI: RentCarAPI,
) : RentCarDataSource {
    override suspend fun getUnassignedCar(rentSchedule: RequestUnassignedCar): RentCar {
        return rentCarAPI.getUnassignedCar(rentSchedule)
    }

    override suspend fun getOffCar(rentInfo: RequestCarStatus): String {
        return rentCarAPI.getOffCar(rentInfo)
    }

    override suspend fun getOnCar(rentInfo: RequestCarStatus): String {
        return rentCarAPI.getOnCar(rentInfo)
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
}