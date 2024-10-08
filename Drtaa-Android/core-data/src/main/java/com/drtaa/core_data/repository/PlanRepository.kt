package com.drtaa.core_data.repository

import com.drtaa.core_model.plan.LastPlan
import com.drtaa.core_model.plan.Plan
import com.drtaa.core_model.plan.PlanItem
import com.drtaa.core_model.plan.PlanSimple
import com.drtaa.core_model.plan.RequestPlanName
import com.drtaa.core_model.plan.ResponsePutPlan
import kotlinx.coroutines.flow.Flow

interface PlanRepository {
    suspend fun putPlan(plan: Plan): Flow<Result<ResponsePutPlan>>
    suspend fun getPlanList(): Flow<Result<List<PlanSimple>>>
    suspend fun getPlanDetail(travelId: Int): Flow<Result<Plan>>
    suspend fun updatePlanName(planName: RequestPlanName): Flow<Result<String>>
    suspend fun getTodayPlanList(): Flow<Result<List<PlanItem>>>
    suspend fun addPlanAtLast(plan: LastPlan): Flow<Result<String>>
}