package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.datasource.RentDataSource
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_data.util.ResultWrapper
import com.drtaa.core_data.util.safeApiCall
import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.network.ResponseCallRent
import com.drtaa.core_model.rent.RentCar
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import timber.log.Timber
import javax.inject.Inject

class RentRepositoryImpl @Inject constructor(
    private val rentDataSource: RentDataSource
) : RentRepository {
    override suspend fun getUnassignedCar(rentSchedule: RequestUnassignedCar): Flow<Result<RentCar>> =
        flow {
            when (
                val response = safeApiCall { rentDataSource.getUnassignedCar(rentSchedule) }
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

    override suspend fun completeRent(requestCompleteRent: RequestCompleteRent): Flow<Result<Unit>> =
        flow {
            when (
                val response = safeApiCall { rentDataSource.completeRent(requestCompleteRent) }
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

    override suspend fun callRent(requestCallRent: RequestCallRent): Flow<Result<ResponseCallRent>> = flow {
        when (val response = safeApiCall { rentDataSource.callRent(requestCallRent) }) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
                Timber.d("렌트 호출 성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("렌트 호출 실패: ${response.message}")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("렌트 호출 네트워크 에러")
            }
        }
    }
}