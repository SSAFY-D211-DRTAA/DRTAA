package com.drtaa.core_network.api

import com.drtaa.core_model.network.RequestDispatch
import com.drtaa.core_model.network.RequestDrivingCar
import com.drtaa.core_model.network.ResponseRentCar
import retrofit2.http.GET
import retrofit2.http.PATCH
import retrofit2.http.Path

interface RentAPI {
    @GET("rent-car/dispatch/unassigned")
    suspend fun getUnassignedCar(): ResponseRentCar

    @GET("rent-car/dispatch/assigned")
    suspend fun getAssignedCar(): ResponseRentCar

    @GET("rent-car/dispatch/{rentCarId}")
    suspend fun getAssignedCar(@Path("rentCarId") rentCarId: Int): ResponseRentCar

    @GET("rent-car/drive/{rentCarId}")
    suspend fun getDrivingCar(@Path("rentCarId") rentCarId: Int): ResponseRentCar

    @PATCH("rent-car/dispatch")
    suspend fun patchDispatchCar(request: RequestDispatch): ResponseRentCar

    @GET("rent-car/dispatch")
    suspend fun getAllDispatchCar(): ResponseRentCar

    @PATCH("rent-car/drive")
    suspend fun patchDriveCar(request: RequestDrivingCar): ResponseRentCar
}