package com.drtaa.core_network.api

import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.ResponseRentStateAll
import com.drtaa.core_model.rent.RentDetail
import com.drtaa.core_model.rent.RentSimple
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.PATCH
import retrofit2.http.POST

interface RentAPI {
    @GET("rent")
    suspend fun getRentHistory(): List<RentSimple>

    @POST("rent")
    suspend fun callAllRent(
        @Body requestCallRent: RequestCallRent,
    ): RentDetail

    @PATCH("rent/status/completed")
    suspend fun completeRent(
        @Body requestCompleteRent: RequestCompleteRent,
    )

    @GET("rent/status/active")
    suspend fun getAllRentState(): List<ResponseRentStateAll>

    @GET("rent/current")
    suspend fun getCurrentRent(): RentDetail
}