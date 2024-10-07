package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.datasource.PlanDataSource
import com.drtaa.core_data.repository.PlanRepository
import com.drtaa.core_data.util.ResultWrapper
import com.drtaa.core_data.util.safeApiCall
import com.drtaa.core_model.plan.LastPlan
import com.drtaa.core_model.plan.Plan
import com.drtaa.core_model.plan.PlanItem
import com.drtaa.core_model.plan.PlanSimple
import com.drtaa.core_model.plan.RequestPlanName
import com.drtaa.core_model.plan.ResponsePutPlan
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import timber.log.Timber
import javax.inject.Inject

class PlanRepositoryImpl @Inject constructor(
    private val planDataSource: PlanDataSource
) : PlanRepository {
    override suspend fun putPlan(plan: Plan): Flow<Result<ResponsePutPlan>> = flow {
        when (
            val response = safeApiCall { planDataSource.putPlan(plan) }
        ) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
                Timber.d("플랜 수정 성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("플랜 수정 실패: ${response.message}")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("플랜 수정 네트워크 에러")
            }
        }
    }

    override suspend fun getPlanList(): Flow<Result<List<PlanSimple>>> = flow {
        when (
            val response = safeApiCall { planDataSource.getPlanList() }
        ) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
                Timber.d("플랜 목록 조회 성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("플랜 목록 조회 실패: ${response.message}")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("플랜 목록 조회 네트워크 에러")
            }
        }
    }

    override suspend fun getPlanDetail(travelId: Int): Flow<Result<Plan>> = flow {
        when (
            val response = safeApiCall { planDataSource.getPlanDetail(travelId) }
        ) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
                Timber.d("플랜 상세 조회 성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("플랜 상세 조회 실패: ${response.message}")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("플랜 상세 조회 네트워크 에러")
            }
        }
    }

    override suspend fun updatePlanName(planName: RequestPlanName): Flow<Result<String>> = flow {
        when (
            val response = safeApiCall { planDataSource.updatePlanName(planName) }
        ) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
                Timber.d("플랜 이름 수정 성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("플랜 이름 수정 실패: ${response.message}")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("플랜 이름 수정 네트워크 에러")
            }
        }
    }

    override suspend fun getTodayPlanList(): Flow<Result<List<PlanItem>>> = flow {
        when (
            val response = safeApiCall { planDataSource.getTodayPlanList() }
        ) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
                Timber.d("오늘 일정 가져오기 성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("오늘 일정 가져오기 성공")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("네트워크 에러")
            }
        }
    }

    override suspend fun addPlanAtLast(plan: LastPlan): Flow<Result<String>> = flow {
        when(val response = safeApiCall { planDataSource.addPlanAtLast(plan) }){
            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
            }
            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
            }
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
            }
        }
    }


}