package com.drtaa.core_model.plan

data class DayPlan(
    val travelId: Int,
    val placesDetail: List<PlanItem>,
    val travelDatesDate: String,
    val travelDatesId: Int
)
