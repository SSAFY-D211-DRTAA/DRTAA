package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.datasource.MapDataSource
import com.drtaa.core_data.repository.MapRepository
import com.drtaa.core_data.util.ResultWrapper
import com.drtaa.core_data.util.safeApiCall
import com.drtaa.core_model.network.ResponseSearch
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import timber.log.Timber
import javax.inject.Inject

class MapRepositoryImpl @Inject constructor(
    private val mapDataSource: MapDataSource
) : MapRepository {
    override suspend fun getSearchList(keyword: String): Flow<Result<ResponseSearch>> = flow {
        when (
            val response = safeApiCall { mapDataSource.getSearchList(keyword) }
        ) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
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