package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.datasource.NaverDataSource
import com.drtaa.core_data.repository.NaverRepository
import com.drtaa.core_data.util.ResultWrapper
import com.drtaa.core_data.util.safeApiCall
import com.drtaa.core_model.map.Search
import com.drtaa.core_model.util.toSearch
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import timber.log.Timber
import javax.inject.Inject

class NaverRepositoryImpl @Inject constructor(
    private val naverDataSource: NaverDataSource
) : NaverRepository {
    override suspend fun getSearchList(keyword: String): Flow<Result<List<Search>>> = flow {
        when (
            val response = safeApiCall { naverDataSource.getSearchList(keyword) }
        ) {
            is ResultWrapper.Success -> {
                emit(
                    Result.success(response.data.items.map { it.toSearch() })
                )
                Timber.d("성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("실패")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("네트워크 에러")
            }
        }
    }
}