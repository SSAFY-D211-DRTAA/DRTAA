package com.drtaa.core_network.api

import com.drtaa.core_model.network.ResponseTour
import retrofit2.http.GET
import retrofit2.http.Query

interface TourAPI {
    @GET("locationBasedList1")
    suspend fun getLocationBasedList(
        @Query("arrange") arrange: String = "R",
        @Query("serviceKey") serviceKey: String,
        @Query("numOfRows") numOfRows: Int,
        @Query("pageNo") pageNo: Int,
        @Query("MobileOS") mobileOS: String,
        @Query("MobileApp") mobileApp: String,
        @Query("_type") type: String,
        @Query("mapX") mapX: String,
        @Query("mapY") mapY: String,
        @Query("radius") radius: String,
    ): ResponseTour


}