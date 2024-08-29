package com.drtaa.core_network.api

import retrofit2.http.GET

interface TestAPI {
    @GET("")
    suspend fun getTest()
}