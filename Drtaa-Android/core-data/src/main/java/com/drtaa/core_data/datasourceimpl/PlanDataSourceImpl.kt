package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.PlanDataSource
import com.drtaa.core_model.plan.Plan
import com.drtaa.core_model.plan.PlanSimple
import com.drtaa.core_model.plan.RequestPlanName
import com.drtaa.core_model.plan.ResponsePutPlan
import com.drtaa.core_network.api.PlanAPI
import javax.inject.Inject

class PlanDataSourceImpl @Inject constructor(
    private val planAPI: PlanAPI
) : PlanDataSource {
    override suspend fun putPlan(plan: Plan): ResponsePutPlan {
        return planAPI.putPlan(plan)
    }

    override suspend fun getPlanList(): List<PlanSimple> {
        return planAPI.getPlanList()
    }

    override suspend fun getPlanDetail(travelId: Int): Plan {
        return planAPI.getPlanDetail(travelId)
    }

    override suspend fun updatePlanName(planName: RequestPlanName): String {
        return planAPI.updatePlanName(planName)
    }
}