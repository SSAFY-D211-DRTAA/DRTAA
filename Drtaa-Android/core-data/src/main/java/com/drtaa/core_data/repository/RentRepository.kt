package com.drtaa.core_data.repository

import com.drtaa.core_model.network.ResponseRentCar
import kotlinx.coroutines.flow.Flow

interface RentRepository {
    suspend fun getUnassignedCar(): Flow<Result<ResponseRentCar>>
}