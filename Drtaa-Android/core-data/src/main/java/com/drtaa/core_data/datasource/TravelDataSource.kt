package com.drtaa.core_data.datasource

import com.drtaa.core_model.travel.NaverPost
import com.drtaa.core_model.travel.ResponseNaverImage
import com.drtaa.core_model.travel.Weather

interface TravelDataSource {
    suspend fun getBlogPostList(keyword: String): List<NaverPost>
    suspend fun getWeatherList(lat: Double, lon: Double): List<Weather>
    suspend fun getImageList(keyword: String): ResponseNaverImage
}
