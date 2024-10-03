package com.drtaa.core_network.api

import com.drtaa.core_model.taxi.RequestCallTaxi
import com.drtaa.core_model.taxi.TaxiDetail
import retrofit2.http.Body
import retrofit2.http.POST

interface TaxiAPI {
    // 택시 api
    @POST("taxi")
    suspend fun callTaxi(
        @Body requestCallTaxi: RequestCallTaxi,
    ): TaxiDetail
}