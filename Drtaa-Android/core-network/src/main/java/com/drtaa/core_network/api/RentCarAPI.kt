package com.drtaa.core_network.api

import com.drtaa.core_model.network.RequestCarStatus
import com.drtaa.core_model.network.RequestDrivingCar
import com.drtaa.core_model.network.RequestRentCarCall
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.network.ResponseDrivingCar
import com.drtaa.core_model.network.ResponseRentCarCall
import com.drtaa.core_model.rent.RentCar
import com.drtaa.core_model.rent.RentTravelInfo
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

    @PATCH("rent-car/parking")
    suspend fun getOffCar(
        @Body rentInfo: RequestCarStatus,
    ): RentTravelInfo

    @PATCH("rent-car/driving")
    suspend fun getOnCar(
        @Body rentInfo: RequestCarStatus,
    ): RentTravelInfo

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

    @GET("rent-car/{rentCarId}")
    suspend fun getDriveStatus(
        @Path("rentCarId") rentCarId: Long,
    ): ResponseDrivingCar
}