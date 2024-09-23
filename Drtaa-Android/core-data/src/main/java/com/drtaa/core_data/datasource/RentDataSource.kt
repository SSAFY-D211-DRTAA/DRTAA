package com.drtaa.core_data.datasource

import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.RequestDuplicatedSchedule
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.rent.RentCar
import com.drtaa.core_model.rent.RentDetail
import com.drtaa.core_model.rent.RentSimple

interface RentDataSource {
    suspend fun getUnassignedCar(rentSchedule: RequestUnassignedCar): RentCar
    suspend fun callRent(requestCallRent: RequestCallRent): RentDetail
    suspend fun completeRent(requestCompleteRent: RequestCompleteRent)
    suspend fun getRentHistory(): List<RentSimple>
    suspend fun getCurrentRent(): RentDetail
    suspend fun checkDuplicatedRent(rentSchedule: RequestDuplicatedSchedule): Boolean
}