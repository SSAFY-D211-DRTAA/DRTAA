package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.TravelDataSource
import com.drtaa.core_model.travel.NaverPost
import com.drtaa.core_model.travel.Weather
import com.drtaa.core_network.api.TravelAPI
import javax.inject.Inject

class TravelDataSourceImpl @Inject constructor(
    private val travelAPI: TravelAPI
): TravelDataSource {

    override suspend fun getBlogPostList(keyword: String): List<NaverPost> {
        return travelAPI.getBlogPostList(keyword)
    }

    override suspend fun getWeatherList(lat: Double, lon: Double): List<Weather> {
        return travelAPI.getWeatherList(lat, lon)
    }
}
