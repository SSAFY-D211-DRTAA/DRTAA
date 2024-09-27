package com.drtaa.core_data.datasource

import com.drtaa.core_model.plan.Plan
import com.drtaa.core_model.plan.PlanSimple
import com.drtaa.core_model.plan.RequestPlanName

interface PlanDataSource {
    suspend fun putPlan(plan: Plan): String
    suspend fun getPlanList(): List<PlanSimple>
    suspend fun getPlanDetail(travelId: Int): Plan
    suspend fun updatePlanName(planName: RequestPlanName): String
}