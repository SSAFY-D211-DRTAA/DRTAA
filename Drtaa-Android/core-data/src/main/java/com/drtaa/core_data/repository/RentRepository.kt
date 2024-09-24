package com.drtaa.core_data.repository

import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestChangeRent
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.RequestRentExtend
import com.drtaa.core_model.network.ResponseRentStateAll
import com.drtaa.core_model.rent.RentDetail
import com.drtaa.core_model.rent.RentSimple
import kotlinx.coroutines.flow.Flow

interface RentRepository {
    // rent
    suspend fun callRent(requestCallRent: RequestCallRent): Flow<Result<RentDetail>>
    suspend fun changeRent(requestChangeRent: RequestChangeRent): Flow<Result<String>>
    suspend fun extendRentTime(requestRentExtend: RequestRentExtend): Flow<Result<String>>
    suspend fun getAllCompletedRent(rentId: Long): Flow<Result<List<ResponseRentStateAll>>>
    suspend fun completeRent(requestCompleteRent: RequestCompleteRent): Flow<Result<Unit>>
    suspend fun cancelRent(requestCompleteRent: RequestCompleteRent): Flow<Result<String>>
    suspend fun getRentDetail(rentId: Long): Flow<Result<RentDetail>>
    suspend fun getAllRentState(): Flow<Result<Long>>
    suspend fun getCurrentRent(): Flow<Result<RentDetail>>

    // history
    suspend fun getRentHistory(): Flow<Result<List<RentSimple>>>
}