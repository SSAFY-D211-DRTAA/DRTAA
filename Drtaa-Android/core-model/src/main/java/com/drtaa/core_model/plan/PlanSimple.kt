package com.drtaa.core_model.plan

data class PlanSimple(
    val travelId: Int,
    val travelEndDate: String,
    val travelName: String,
    val travelStartDate: String,
    val travelStatus: String
)