package com.drtaa.core_model.map

data class ResponseDest(
    val tag: String,
    val msg: Destination,
)

data class Destination(
    val name: String?,
    val latitude: Double,
    val longitude: Double,
    val remainingDistance: Int,
    val remainingTime: Int
)