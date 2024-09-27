package com.drtaa.core_model.plan

data class PlanSimple(
    val rentId: Int,
    val rentStatus: String,
    val travelId: Int,
    val travelEndDate: String,
    val travelName: String,
    val travelStartDate: String
)