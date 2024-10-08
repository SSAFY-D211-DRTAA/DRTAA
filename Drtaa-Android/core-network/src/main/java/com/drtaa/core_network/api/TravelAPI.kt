package com.drtaa.core_network.api

import com.drtaa.core_model.travel.NaverPost
import com.drtaa.core_model.travel.ResponseNaverImage
import com.drtaa.core_model.travel.Weather
import com.drtaa.core_network.BuildConfig
import retrofit2.http.GET
import retrofit2.http.Header
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

    @GET("image")
    suspend fun getImageList(
        @Header("X-Naver-Client-Id") clientId: String = BuildConfig.NAVER_CLIENT_ID,
        @Header("X-Naver-Client-Secret") clientSecret: String = BuildConfig.NAVER_CLIENT_SECRET,
        @Query("query") keyword: String
    ): ResponseNaverImage
}