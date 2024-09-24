package com.drtaa.core_network.api

import com.drtaa.core_model.network.RequestDrivingCar
import com.drtaa.core_model.network.RequestRentCarCall
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.network.ResponseDrivingCar
import com.drtaa.core_model.network.ResponseRentCarCall
import com.drtaa.core_model.rent.RentCar
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.PATCH
import retrofit2.http.POST
import retrofit2.http.Path

interface RentCarAPI {
    @POST("rent-car/dispatch")
    suspend fun getUnassignedCar(
        @Body rentSchedule: RequestUnassignedCar,
    ): RentCar

    @PATCH("rent-car/{rentId}/parking")
    suspend fun getOffCar(
        @Path("rentId") rentId: Long,
    ): String

    @PATCH("rent-car/{rentId}/driving")
    suspend fun getOnCar(
        @Path("rentId") rentId: Long,
    ): String

    @PATCH("rent-car/call")
    suspend fun callAssignedCar(
        @Body requestCallCar: RequestRentCarCall,
    ): ResponseRentCarCall

    @PATCH("rent-car/call/{rentId}")
    suspend fun callFirstAssignedCar(
        @Path("rentId") rentId: Long,
    ): ResponseRentCarCall

    @PATCH("rent-car/drive")
    suspend fun editDriveStatus(
        @Body request: RequestDrivingCar,
    ): ResponseDrivingCar

    @GET("rent-car/drive/{rentCarId}")
    suspend fun getDriveStatus(
        @Path("rentCarId") rentCarId: Long,
    ): ResponseDrivingCar
}