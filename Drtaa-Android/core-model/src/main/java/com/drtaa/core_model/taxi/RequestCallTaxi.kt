package com.drtaa.core_model.taxi

data class RequestCallTaxi(
    val taxiTime: Int,
    val taxiPrice: Int,
    val taxiStartTime: String,
    val taxiEndTime: String,
    val taxiStartLat: Double,
    val taxiStartLon: Double,
    val taxiEndLat: Double,
    val taxiEndLon: Double,
    val startAddress: String,
    val endAddress: String,
)
