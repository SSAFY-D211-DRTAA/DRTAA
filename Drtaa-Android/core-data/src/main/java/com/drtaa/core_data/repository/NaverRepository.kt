package com.drtaa.core_data.repository

import com.drtaa.core_model.data.Search
import kotlinx.coroutines.flow.Flow

interface NaverRepository {
    suspend fun getSearchList(keyword: String): Flow<Result<List<Search>>>
}