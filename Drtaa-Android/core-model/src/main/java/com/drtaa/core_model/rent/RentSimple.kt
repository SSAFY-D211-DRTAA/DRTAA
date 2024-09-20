package com.drtaa.core_model.rent

data class RentSimple(
    val rentHeadCount: Int,
    val rentId: Int,
    val rentStartTime: String,
    val rentStatus: String,
    val rentTime: Int
)