package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.datasource.RentDataSource
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_data.util.ResultWrapper
import com.drtaa.core_data.util.safeApiCall
import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestCarStatus
import com.drtaa.core_model.network.RequestChangeRent
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.RequestDuplicatedSchedule
import com.drtaa.core_model.network.RequestRentExtend
import com.drtaa.core_model.network.ResponseRentStateAll
import com.drtaa.core_model.rent.RentDetail
import com.drtaa.core_model.rent.RentSimple
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import timber.log.Timber
import javax.inject.Inject

class RentRepositoryImpl @Inject constructor(
    private val rentDataSource: RentDataSource,
) : RentRepository {

    override suspend fun completeRent(requestCompleteRent: RequestCompleteRent): Flow<Result<String>> =
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

    override suspend fun cancelRent(requestCompleteRent: RequestCompleteRent): Flow<Result<String>> =
        flow {
            when (val response = safeApiCall { rentDataSource.cancelRent(requestCompleteRent) }) {
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

    override suspend fun getRentDetail(rentId: Long): Flow<Result<RentDetail>> = flow {
        when (val response = safeApiCall { rentDataSource.getRentDetail(rentId) }) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
                Timber.d("렌트 상세 조회 성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("렌트 상세 조회 실패: ${response.message}")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("렌트 상세 조회 네트워크 에러")
            }
        }
    }

    override suspend fun callRent(requestCallRent: RequestCallRent): Flow<Result<RentDetail>> =
        flow {
            when (
                val response = safeApiCall { rentDataSource.callRent(requestCallRent) }
            ) {
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

    override suspend fun changeRent(requestChangeRent: RequestChangeRent): Flow<Result<String>> =
        flow {
            when (val response = safeApiCall { rentDataSource.changeRent(requestChangeRent) }) {
                is ResultWrapper.Success -> {
                    emit(Result.success(response.data))
                    Timber.d("렌트 변경 성공")
                }

                is ResultWrapper.GenericError -> {
                    emit(Result.failure(Exception(response.message)))
                    Timber.d("렌트 변경 실패: ${response.message}")
                }

                is ResultWrapper.NetworkError -> {
                    emit(Result.failure(Exception("네트워크 에러")))
                    Timber.d("렌트 변경 네트워크 에러")
                }
            }
        }

    override suspend fun extendRentTime(requestRentExtend: RequestRentExtend): Flow<Result<String>> =
        flow {
            when (val response = safeApiCall { rentDataSource.extendRentTime(requestRentExtend) }) {
                is ResultWrapper.Success -> {
                    emit(Result.success(response.data))
                    Timber.d("렌트 시간 연장 성공")
                }

                is ResultWrapper.GenericError -> {
                    emit(Result.failure(Exception(response.message)))
                    Timber.d("렌트 시간 연장 실패: ${response.message}")
                }

                is ResultWrapper.NetworkError -> {
                    emit(Result.failure(Exception("네트워크 에러")))
                    Timber.d("렌트 시간 연장 네트워크 에러")
                }
            }
        }

    override suspend fun getAllCompletedRent(rentId: Long): Flow<Result<List<ResponseRentStateAll>>> =
        flow {
            when (val response = safeApiCall { rentDataSource.getAllCompletedRent(rentId) }) {
                is ResultWrapper.Success -> {
                    emit(Result.success(response.data))
                    Timber.d("렌트 완료 내역 조회 성공")
                }

                is ResultWrapper.GenericError -> {
                    emit(Result.failure(Exception(response.message)))
                    Timber.d("렌트 완료 내역 조회 실패: ${response.message}")
                }

                is ResultWrapper.NetworkError -> {
                    emit(Result.failure(Exception("네트워크 에러")))
                    Timber.d("렌트 완료 내역 조회 네트워크 에러")
                }
            }
        }

    override suspend fun getRentHistory(): Flow<Result<List<RentSimple>>> = flow {
        when (
            val response = safeApiCall { rentDataSource.getRentHistory() }
        ) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
                Timber.d("렌트 이용 기록 조회 성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("렌트 이용 기록 조회 실패: ${response.message}")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("렌트 이용 기록 조회 네트워크 에러")
            }
        }
    }

    override suspend fun getCurrentRent(): Flow<Result<RentDetail>> = flow {
        when (
            val response = safeApiCall { rentDataSource.getCurrentRent() }
        ) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
                Timber.d("현재 렌트 조회 성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("현재 렌트 조회 실패: ${response.message}")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("현재 렌트 조회 네트워크 에러")
            }
        }
    }

    override suspend fun completeTodayRent(rentInfo: RequestCarStatus): Flow<Result<String>> = flow {
        when(
            val response = safeApiCall { rentDataSource.completeTodayRent(rentInfo) }
        ){
            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("오늘 렌트 완료 실패: ${response.message}")
            }
            ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("오늘 렌트 완료 네트워크 에러")
            }
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
                Timber.d("오늘 렌트 완료 성공")
            }
        }
    }

    override suspend fun getReservedRentState(): Flow<Result<ResponseRentStateAll>> = flow {
        when (
            val response = safeApiCall { rentDataSource.getAllRentState() }
        ) {
            is ResultWrapper.Success -> {
                val result = response.data
                if (result.isEmpty()) {
                    emit(Result.failure(Exception("예약 내역 없음")))
                } else {
                    // 가장 가까운 예약내역 긁어옴
                    emit(Result.success(result.first()))
                }
                Timber.d("전체 렌트/내역 조회 성공 ${response.data}")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("전체 렌트/내역 조회 실패: ${response.message}")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("전체 렌트/내역 조회 네트워크 에러")
            }
        }
    }

    override suspend fun checkDuplicatedRent(rentSchedule: RequestDuplicatedSchedule): Flow<Result<Boolean>> =
        flow {
            when (
                val response = safeApiCall { rentDataSource.checkDuplicatedRent(rentSchedule) }
            ) {
                is ResultWrapper.Success -> {
                    emit(Result.success(response.data))
                    Timber.d("스케줄 중복 확인 성공")
                }

                is ResultWrapper.GenericError -> {
                    emit(Result.failure(Exception(response.message)))
                    Timber.d("스케줄 중복 확인 실패: ${response.message}")
                }

                is ResultWrapper.NetworkError -> {
                    emit(Result.failure(Exception("네트워크 에러")))
                    Timber.d("스케줄 중복 확인 네트워크 에러")
                }
            }
        }
}