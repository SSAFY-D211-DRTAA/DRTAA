package com.drtaa.core_data.repository

import com.drtaa.core_model.map.Search
import com.drtaa.core_model.route.ResponseGeoJson
import com.drtaa.core_model.taxi.RequestCallTaxi
import com.drtaa.core_model.taxi.TaxiDetail
import kotlinx.coroutines.flow.Flow

interface TaxiRepository {
    suspend fun getRoute(start: Search, end: Search): Flow<Result<ResponseGeoJson>>
    suspend fun callTaxi(requestCallTaxi: RequestCallTaxi): Flow<Result<TaxiDetail>>
}