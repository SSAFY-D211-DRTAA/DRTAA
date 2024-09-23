package com.drtaa.core_network.api

import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.rent.RentCar
import com.drtaa.core_model.rent.RentDetail
import com.drtaa.core_model.rent.RentSimple
import retrofit2.http.Body
import retrofit2.http.GET
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
    ): RentDetail

    @PATCH("rent/status/completed")
    suspend fun completeRent(
        @Body requestCompleteRent: RequestCompleteRent
    )

    @GET("rent")
    suspend fun getRentHistory(): List<RentSimple>

    @GET("rent/current")
    suspend fun getCurrentRent(): RentDetail
}