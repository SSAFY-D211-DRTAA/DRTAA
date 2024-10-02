package com.drtaa.core_data.datasource

import com.drtaa.core_model.network.ResponseReverseGeocode
import com.drtaa.core_model.network.ResponseSearch

interface NaverDataSource {
    suspend fun getSearchList(keyword: String): ResponseSearch
    suspend fun getReverseGeocode(latitude: Double, longitude: Double): ResponseReverseGeocode
}
