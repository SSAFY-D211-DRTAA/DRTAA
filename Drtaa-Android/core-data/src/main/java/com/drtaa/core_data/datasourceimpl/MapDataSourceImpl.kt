package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.MapDataSource
import com.drtaa.core_model.network.ResponseSearch
import com.drtaa.core_network.api.MapAPI
import javax.inject.Inject

class MapDataSourceImpl @Inject constructor(
    private val mapAPI: MapAPI
) : MapDataSource {
    override suspend fun getSearchList(keyword: String): ResponseSearch {
        return mapAPI.getSearchList(keyword = keyword)
    }
}