package com.drtaa.core_data.repository

import com.drtaa.core_model.map.Search
import com.drtaa.core_model.network.ResponseReverseGeocode
import kotlinx.coroutines.flow.Flow

interface NaverRepository {
    suspend fun getSearchList(keyword: String): Flow<Result<List<Search>>>
    suspend fun getReverseGeocode(latitude: Double, longitude: Double): Flow<Result<ResponseReverseGeocode>>
}