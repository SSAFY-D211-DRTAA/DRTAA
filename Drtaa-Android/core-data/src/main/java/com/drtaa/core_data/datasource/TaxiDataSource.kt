package com.drtaa.core_data.datasource

import com.drtaa.core_model.map.Search
import com.drtaa.core_model.route.ResponseGeoJson
import com.drtaa.core_model.taxi.RequestCallTaxi
import com.drtaa.core_model.taxi.TaxiDetail

interface TaxiDataSource {
    suspend fun getRoute(start: Search, end: Search): ResponseGeoJson
    suspend fun callTaxi(requestCallTaxi: RequestCallTaxi): TaxiDetail
}