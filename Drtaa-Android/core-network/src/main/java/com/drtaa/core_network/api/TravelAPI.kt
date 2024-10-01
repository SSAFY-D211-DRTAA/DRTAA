
package com.drtaa.core_network.api

import com.drtaa.core_model.travel.NaverPost
import com.drtaa.core_model.travel.Weather
import retrofit2.http.GET
import retrofit2.http.Path
import retrofit2.http.Query

interface TravelAPI {
    @GET("travel/blog/{keyword}")
    suspend fun getBlogPostList(
        @Path("keyword") keyword: String
    ): List<NaverPost>

    @GET("travel/weather")
    suspend fun getWeatherList(
        @Query("datePlacesLat") lat: Double,
        @Query("datePlacesLon") lon: Double
    ): List<Weather>
}