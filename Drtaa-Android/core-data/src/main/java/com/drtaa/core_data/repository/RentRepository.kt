package com.drtaa.core_data.repository

import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.rent.CarPosition
import com.drtaa.core_model.rent.RentCar
import com.drtaa.core_model.rent.RentDetail
import com.drtaa.core_model.rent.RentSimple
import kotlinx.coroutines.flow.Flow

interface RentRepository {
    suspend fun getUnassignedCar(rentSchedule: RequestUnassignedCar): Flow<Result<RentCar>>
    suspend fun callRent(requestCallRent: RequestCallRent): Flow<Result<RentDetail>>
    suspend fun completeRent(requestCompleteRent: RequestCompleteRent): Flow<Result<Unit>>
    suspend fun getRentHistory(): Flow<Result<List<RentSimple>>>
    suspend fun getCurrentRent(): Flow<Result<RentDetail>>
    suspend fun callAssignedCar(
        rentId: Long,
        userLat: Double,
        userLon: Double
    ): Flow<Result<CarPosition>>

    suspend fun getAllRentState(): Flow<Result<Long>>
}