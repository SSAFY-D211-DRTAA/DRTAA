package com.drtaa.core_data.datasource

import com.drtaa.core_model.network.RequestCarStatus
import com.drtaa.core_model.network.RequestDrivingCar
import com.drtaa.core_model.network.RequestRentCarCall
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.network.ResponseDrivingCar
import com.drtaa.core_model.network.ResponseRentCarCall
import com.drtaa.core_model.rent.RentCar
import com.drtaa.core_model.rent.RentTravelInfo

interface RentCarDataSource {
    // rent-car
    suspend fun getUnassignedCar(rentSchedule: RequestUnassignedCar): RentCar
    suspend fun getOffCar(rentInfo: RequestCarStatus): String
    suspend fun getOnCar(rentInfo: RequestCarStatus): RentTravelInfo
    suspend fun callAssignedCar(requestCallCar: RequestRentCarCall): ResponseRentCarCall
    suspend fun callFirstAssignedCar(rentId: Long): ResponseRentCarCall
    suspend fun editDriveStatus(request: RequestDrivingCar): ResponseDrivingCar
    suspend fun getDriveStatus(rentCarId: Long): ResponseDrivingCar
    suspend fun getCarWithTravelInfo(): RequestCarStatus
    suspend fun setCarWithTravelInfo(rentInfo: RequestCarStatus)
}