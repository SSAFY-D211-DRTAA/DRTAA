package com.drtaa.core_data.datasource

import com.drtaa.core_model.network.RequestDrivingCar
import com.drtaa.core_model.network.RequestRentCarCall
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.network.ResponseDrivingCar
import com.drtaa.core_model.network.ResponseRentCarCall
import com.drtaa.core_model.rent.RentCar

interface RentCarDataSource {
    // rent-car
    suspend fun getUnassignedCar(rentSchedule: RequestUnassignedCar): RentCar
    suspend fun getOffCar(rentId: Long): String
    suspend fun getOnCar(rentId: Long): String
    suspend fun callAssignedCar(requestCallCar: RequestRentCarCall): ResponseRentCarCall
    suspend fun callFirstAssignedCar(rentId: Long): ResponseRentCarCall
    suspend fun editDriveStatus(request: RequestDrivingCar): ResponseDrivingCar
    suspend fun getDriveStatus(rentCarId: Long): ResponseDrivingCar
}