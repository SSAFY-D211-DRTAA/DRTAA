package com.drtaa.core_data.datasource

import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestCarStatus
import com.drtaa.core_model.network.RequestChangeRent
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.RequestDuplicatedSchedule
import com.drtaa.core_model.network.RequestRentExtend
import com.drtaa.core_model.network.ResponseRentStateAll
import com.drtaa.core_model.rent.RentDetail
import com.drtaa.core_model.rent.RentSimple

interface RentDataSource {
    // rent
    suspend fun callRent(requestCallRent: RequestCallRent): RentDetail
    suspend fun changeRent(requestChangeRent: RequestChangeRent): String
    suspend fun extendRentTime(requestRentExtend: RequestRentExtend): String
    suspend fun getAllCompletedRent(rentId: Long): List<ResponseRentStateAll>
    suspend fun completeRent(requestCompleteRent: RequestCompleteRent): String
    suspend fun cancelRent(requestCompleteRent: RequestCompleteRent): String
    suspend fun getRentDetail(rentId: Long): RentDetail
    suspend fun getAllRentState(): List<ResponseRentStateAll>
    suspend fun getCurrentRent(): RentDetail
    suspend fun checkDuplicatedRent(rentSchedule: RequestDuplicatedSchedule): Boolean
    suspend fun completeTodayRent(rentInfo: RequestCarStatus): String

    // history
    suspend fun getRentHistory(): List<RentSimple>
}