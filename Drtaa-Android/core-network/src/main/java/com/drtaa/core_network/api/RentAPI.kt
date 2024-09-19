package com.drtaa.core_network.api

import com.drtaa.core_model.network.ResponseRentCar
import retrofit2.http.GET

interface RentAPI {
    @GET("rent-car/dispatch/unassigned")
    suspend fun getUnassignedCar(): ResponseRentCar
}