package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.datasource.RentCarDataSource
import com.drtaa.core_data.repository.RentCarRepository
import com.drtaa.core_data.util.ResultWrapper
import com.drtaa.core_data.util.safeApiCall
import com.drtaa.core_model.network.RequestDrivingCar
import com.drtaa.core_model.network.RequestRentCarCall
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.network.ResponseDrivingCar
import com.drtaa.core_model.rent.CarPosition
import com.drtaa.core_model.rent.RentCar
import com.drtaa.core_model.util.toCarPosition
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import timber.log.Timber
import javax.inject.Inject

class RentCarRepositoryImpl @Inject constructor(
    private val rentCarDataSource: RentCarDataSource,
) : RentCarRepository {
    override suspend fun getUnassignedCar(rentSchedule: RequestUnassignedCar): Flow<Result<RentCar>> =
        flow {
            when (
                val response = safeApiCall { rentCarDataSource.getUnassignedCar(rentSchedule) }
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

    override suspend fun callAssignedCar(
        rentId: Long,
        userLat: Double,
        userLon: Double,
    ): Flow<Result<CarPosition>> =
        flow {
            when (
                val response = safeApiCall {
                    rentCarDataSource.callAssignedCar(
                        RequestRentCarCall(
                            rentId,
                            userLat,
                            userLon
                        )
                    )
                }
            ) {
                is ResultWrapper.Success -> {
                    emit(Result.success(response.data.toCarPosition()))
                    Timber.d("현재 렌트카 호출 성공")
                }

                is ResultWrapper.GenericError -> {
                    emit(Result.failure(Exception(response.message)))
                    Timber.d("현재 렌트카 호출 실패: ${response.message}")
                }

                is ResultWrapper.NetworkError -> {
                    emit(Result.failure(Exception("네트워크 에러")))
                    Timber.d("현재 렌트카 호출 네트워크 에러")
                }
            }
        }

    override suspend fun callFirstAssignedCar(rentId: Long): Flow<Result<CarPosition>> = flow {
        when (val response = safeApiCall { rentCarDataSource.callFirstAssignedCar(rentId) }) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data.toCarPosition()))
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("첫번째 렌트카 호출 실패: ${response.message}")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
            }
        }
    }

    override suspend fun editDriveStatus(
        rentCarId: Long,
        driveStatus: String,
    ): Flow<Result<ResponseDrivingCar>> = flow {
        when (
            val response = safeApiCall {
                rentCarDataSource.editDriveStatus(RequestDrivingCar(rentCarId, driveStatus))
            }
        ) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
                Timber.d("드라이브 상태 변경 성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("드라이브 상태 변경 실패: ${response.message}")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("드라이브 상태 변경 네트워크 에러")
            }
        }
    }

    override suspend fun getDriveStatus(rentCarId: Long): Flow<Result<ResponseDrivingCar>> = flow {
        when (
            val response = safeApiCall { rentCarDataSource.getDriveStatus(rentCarId) }
        ) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
                Timber.d("드라이브 상태 조회 성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("드라이브 상태 조회 실패: ${response.message}")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("드라이브 상태 조회 네트워크 에러")
            }
        }
    }
}