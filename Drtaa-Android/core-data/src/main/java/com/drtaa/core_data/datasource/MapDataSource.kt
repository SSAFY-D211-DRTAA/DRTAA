package com.drtaa.core_data.datasource

import com.drtaa.core_model.network.ResponseSearch

interface MapDataSource {
    suspend fun getSearchList(keyword: String): ResponseSearch
}
