package com.drtaa.core_data.repository

import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.network.ResponseDrivingCar
import com.drtaa.core_model.rent.CarPosition
import com.drtaa.core_model.rent.RentCar
import kotlinx.coroutines.flow.Flow

interface RentCarRepository {
    // rent-car
    suspend fun getUnassignedCar(rentSchedule: RequestUnassignedCar): Flow<Result<RentCar>>
    suspend fun getOffCar(rentId: Long): Flow<Result<String>>
    suspend fun getOnCar(rentId: Long): Flow<Result<String>>
    suspend fun callAssignedCar(
        rentId: Long,
        userLat: Double,
        userLon: Double
    ): Flow<Result<CarPosition>>
    suspend fun callFirstAssignedCar(rentId: Long): Flow<Result<CarPosition>>
    suspend fun editDriveStatus(rentCarId: Long, driveStatus: String): Flow<Result<ResponseDrivingCar>>
    suspend fun getDriveStatus(rentCarId: Long): Flow<Result<ResponseDrivingCar>>
}