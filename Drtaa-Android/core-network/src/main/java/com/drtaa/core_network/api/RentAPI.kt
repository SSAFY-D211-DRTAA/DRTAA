package com.drtaa.core_network.api

import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.network.ResponseCallRent
import com.drtaa.core_model.rent.RentCar
import retrofit2.http.Body
import retrofit2.http.PATCH
import retrofit2.http.POST

interface RentAPI {
    @POST("rent-car/dispatch")
    suspend fun getUnassignedCar(
        @Body rentSchedule: RequestUnassignedCar
    ): RentCar

    @POST("rent")
    suspend fun callRent(
        @Body requestCallRent: RequestCallRent
    ): ResponseCallRent

    @PATCH("rent/status/completed")
    suspend fun completeRent(
        @Body requestCompleteRent: RequestCompleteRent
    ): Unit
}