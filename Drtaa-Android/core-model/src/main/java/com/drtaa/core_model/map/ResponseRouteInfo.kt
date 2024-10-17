package com.drtaa.core_model.map

data class ResponseRouteInfo(
    val tag: String,
    val msg: Info,
)

data class Info(
    val leftTime: String,
    val leftDistance: String,
)