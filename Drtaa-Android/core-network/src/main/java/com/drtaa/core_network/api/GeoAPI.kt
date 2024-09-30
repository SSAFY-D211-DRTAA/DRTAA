package com.drtaa.core_network.api

import com.drtaa.core_model.network.ResponseReverseGeocode
import com.drtaa.core_network.BuildConfig
import retrofit2.http.GET
import retrofit2.http.Header
import retrofit2.http.Query

interface GeoAPI {
    @GET("map-reversegeocode/v2/gc")
    suspend fun getReverseGeocode(
        @Header("X-NCP-APIGW-API-KEY-ID") clientId: String = BuildConfig.NAVER_MAP_CLIENT_ID,
        @Header("X-NCP-APIGW-API-KEY") clientSecret: String = BuildConfig.NAVER_MAP_CLIENT_SECRET,
        @Query("coords") coords: String,
        @Query("output") output: String = "json",
        @Query("orders") orders: String = "roadaddr"
    ): ResponseReverseGeocode
}