package com.drtaa.core_model.plan

data class Plan(
    val travelId: Int,
    val datesDetail: List<DayPlan>,
    val travelEndDate: String,
    val travelName: String,
    val travelStartDate: String
)