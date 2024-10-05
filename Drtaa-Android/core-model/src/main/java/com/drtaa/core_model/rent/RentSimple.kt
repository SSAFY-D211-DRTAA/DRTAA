package com.drtaa.core_model.rent

data class RentSimple(
    val rentId: Int,
    val travelName: String,
    val rentHeadCount: Int,
    val rentStatus: String,
    val rentTime: Int,
    val rentPrice: Int,
    val rentStartTime: String,
    val rentEndTime: String
)