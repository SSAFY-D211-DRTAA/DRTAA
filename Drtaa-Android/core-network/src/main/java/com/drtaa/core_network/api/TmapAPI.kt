package com.drtaa.core_network.api

import com.drtaa.core_model.route.ResponseGeoJson
import com.drtaa.core_network.BuildConfig
import retrofit2.http.Header
import retrofit2.http.POST
import retrofit2.http.Query

interface TmapAPI {
    @POST("tmap/routes")
    suspend fun getRoute(
        @Header("appKey") appKey: String = BuildConfig.APP_KEY,
        @Query("version") version: Int = 1,
        @Query("callback") callback: String? = null,
        @Query("startX") startX: Double,
        @Query("startY") startY: Double,
        @Query("endX") endX: Double,
        @Query("endY") endY: Double,
        @Query("reqCoordType") reqCoordType: String = "WGS84GEO",
        @Query("resCoordType") resCoordType: String = "WGS84GEO",
        @Query("searchOption") searchOption: Int = 0
    ): ResponseGeoJson
}