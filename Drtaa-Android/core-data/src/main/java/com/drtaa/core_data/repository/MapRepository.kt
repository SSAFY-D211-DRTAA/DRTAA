package com.drtaa.core_data.repository

import com.drtaa.core_model.network.ResponseSearch
import kotlinx.coroutines.flow.Flow

interface MapRepository {
    suspend fun getSearchList(keyword: String): Flow<Result<ResponseSearch>>
}