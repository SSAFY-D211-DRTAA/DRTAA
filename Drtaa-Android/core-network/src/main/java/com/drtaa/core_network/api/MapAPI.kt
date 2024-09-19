package com.drtaa.core_network.api

import com.drtaa.core_model.network.ResponseSearch
import com.drtaa.core_network.BuildConfig
import retrofit2.http.GET
import retrofit2.http.Header
import retrofit2.http.Query

interface MapAPI {
    @GET("local.json")
    suspend fun getSearchList(
        @Header("X-Naver-Client-Id") clientId: String = BuildConfig.NAVER_CLIENT_ID,
        @Header("X-Naver-Client-Secret") clientSecret: String = BuildConfig.NAVER_CLIENT_SECRET,
        @Query("query") keyword: String,
        @Query("display") display: Int = 5
    ): ResponseSearch
}