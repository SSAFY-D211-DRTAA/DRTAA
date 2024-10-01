package com.drtaa.core_data.repository

import com.drtaa.core_model.travel.NaverPost
import com.drtaa.core_model.travel.Weather
import kotlinx.coroutines.flow.Flow

interface TravelRepository {
    suspend fun getBlogPostList(keyword: String): Flow<Result<List<NaverPost>>>
    suspend fun getWeatherList(lat: Double, lon: Double): Flow<Result<List<Weather>>>
}