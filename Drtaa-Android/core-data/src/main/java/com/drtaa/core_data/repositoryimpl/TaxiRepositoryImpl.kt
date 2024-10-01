package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.datasource.TaxiDataSource
import com.drtaa.core_data.repository.TaxiRepository
import com.drtaa.core_data.util.ResultWrapper
import com.drtaa.core_data.util.safeApiCall
import com.drtaa.core_model.map.Search
import com.drtaa.core_model.route.ResponseGeoJson
import com.drtaa.core_model.taxi.RequestCallTaxi
import com.drtaa.core_model.taxi.TaxiDetail
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import javax.inject.Inject

class TaxiRepositoryImpl @Inject constructor(
    private val taxiDataSource: TaxiDataSource,
) : TaxiRepository {
    override suspend fun getRoute(start: Search, end: Search): Flow<Result<ResponseGeoJson>> = flow {
        when (val result = safeApiCall { taxiDataSource.getRoute(start, end) }) {
            is ResultWrapper.Success -> {
                emit(Result.success(result.data))
            }
            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(result.message)))
                }
            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
            }
        }
    }

    override suspend fun callTaxi(requestCallTaxi: RequestCallTaxi): Flow<Result<TaxiDetail>> = flow {
        when (val result = safeApiCall { taxiDataSource.callTaxi(requestCallTaxi) }) {
            is ResultWrapper.Success -> {
                emit(Result.success(result.data))
            }
            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(result.message)))
            }
            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
            }
        }
    }
}