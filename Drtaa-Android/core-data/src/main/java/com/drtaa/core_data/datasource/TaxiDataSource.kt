package com.drtaa.core_data.datasource

import com.drtaa.core_model.map.Search
import com.drtaa.core_model.route.ResponseGeoJson

interface TaxiDataSource {
    suspend fun getRoute(start: Search, end: Search): ResponseGeoJson
}