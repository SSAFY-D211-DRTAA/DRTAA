package com.drtaa.core_data.repository

import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.rent.RentCar
import kotlinx.coroutines.flow.Flow

interface RentRepository {
    suspend fun getUnassignedCar(rentSchedule: RequestUnassignedCar): Flow<Result<RentCar>>
    suspend fun completeRent(requestCompleteRent: RequestCompleteRent): Flow<Result<Unit>>
}