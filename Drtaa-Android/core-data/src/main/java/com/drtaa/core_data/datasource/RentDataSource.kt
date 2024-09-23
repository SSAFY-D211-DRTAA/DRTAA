package com.drtaa.core_data.datasource

import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.ResponseRentStateAll
import com.drtaa.core_model.rent.RentDetail
import com.drtaa.core_model.rent.RentSimple

interface RentDataSource : RentCarDataSource {
    // rent
    suspend fun callRent(requestCallRent: RequestCallRent): RentDetail
    suspend fun completeRent(requestCompleteRent: RequestCompleteRent)
    suspend fun getRentHistory(): List<RentSimple>
    suspend fun getCurrentRent(): RentDetail
    suspend fun getAllRentState(): List<ResponseRentStateAll>
}