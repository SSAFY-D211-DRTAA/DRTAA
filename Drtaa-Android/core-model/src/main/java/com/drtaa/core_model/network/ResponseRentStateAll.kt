package com.drtaa.core_model.network

data class ResponseRentStateAll(
    val rentId: Long,
    val rentStatus: String,
    val rentHeadCount: Int,
    val rentTime: Int,
    val rentStartTime: String,
)
