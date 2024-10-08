package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.NaverDataSource
import com.drtaa.core_model.network.ResponseReverseGeocode
import com.drtaa.core_model.network.ResponseSearch
import com.drtaa.core_network.api.GeoAPI
import com.drtaa.core_network.api.MapAPI
import javax.inject.Inject

class NaverDataSourceImpl @Inject constructor(
    private val mapAPI: MapAPI,
    private val geoAPI: GeoAPI
) : NaverDataSource {
    override suspend fun getSearchList(keyword: String): ResponseSearch {
        return mapAPI.getSearchList(keyword = keyword)
    }

    override suspend fun getReverseGeocode(latitude: Double, longitude: Double): ResponseReverseGeocode {
        return geoAPI.getReverseGeocode(coords = "$longitude,$latitude")
    }
}