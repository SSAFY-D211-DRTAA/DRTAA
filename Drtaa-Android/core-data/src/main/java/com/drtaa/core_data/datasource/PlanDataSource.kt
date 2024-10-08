package com.drtaa.core_data.datasource

import com.drtaa.core_model.plan.LastPlan
import com.drtaa.core_model.plan.Plan
import com.drtaa.core_model.plan.PlanItem
import com.drtaa.core_model.plan.PlanSimple
import com.drtaa.core_model.plan.RequestPlanName
import com.drtaa.core_model.plan.ResponsePutPlan

interface PlanDataSource {
    suspend fun putPlan(plan: Plan): ResponsePutPlan
    suspend fun getPlanList(): List<PlanSimple>
    suspend fun getPlanDetail(travelId: Int): Plan
    suspend fun updatePlanName(planName: RequestPlanName): String
    suspend fun getTodayPlanList(): List<PlanItem>
    suspend fun addPlanAtLast(plan: LastPlan): String
}