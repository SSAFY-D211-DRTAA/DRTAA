package com.drtaa.core_network.api

import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestChangeRent
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.RequestRentExtend
import com.drtaa.core_model.network.ResponseRentStateAll
import com.drtaa.core_model.rent.RentDetail
import com.drtaa.core_model.rent.RentSimple
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.PATCH
import retrofit2.http.POST
import retrofit2.http.Path

interface RentAPI {
    @GET("rent")
    suspend fun getRentHistory(): List<RentSimple>

    @POST("rent")
    suspend fun callAllRent(
        @Body requestCallRent: RequestCallRent,
    ): RentDetail

    @PATCH("rent")
    suspend fun changeRent(
        @Body requestChangeRent: RequestChangeRent,
    ): String

    @PATCH("rent/time")
    suspend fun extendRentTime(
        @Body requestRentExtend: RequestRentExtend,
    ): String

    @PATCH("rent/status/{rentId}/in-progress")
    suspend fun getOnRentCar(
        @Path("rentId") rentId: Long,
    ): String

    @GET("rent/status/{rentId}/completed")
    suspend fun getAllCompletedRent(
        @Path("rentId") rentId: Long,
    ): List<ResponseRentStateAll>

    @PATCH("rent/status/completed")
    suspend fun completeRent(
        @Body requestCompleteRent: RequestCompleteRent,
    )

    @PATCH("rent/status/canceled")
    suspend fun cancelRent(
        @Body requestCompleteRent: RequestCompleteRent,
    ): String

    @GET("rent/{rentId}")
    suspend fun getRentDetail(
        @Path("rentId") rentId: Long,
    ): RentDetail

    @GET("rent/status/active")
    suspend fun getAllRentState(): List<ResponseRentStateAll>

    @GET("rent/current")
    suspend fun getCurrentRent(): RentDetail
}