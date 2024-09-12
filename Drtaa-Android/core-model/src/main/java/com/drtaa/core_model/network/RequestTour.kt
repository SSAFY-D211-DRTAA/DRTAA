package com.drtaa.core_model.network

data class RequestTour(
    val numOfRows: Int,
    val pageNo: Int,
    val mapX: String,
    val mapY: String,
    val radius: String,
    val serviceKey: String
)
