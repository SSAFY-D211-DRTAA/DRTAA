package com.drtaa.core_model.taxi

data class TaxiDetail(
    val taxiId: Long,
    val taxiCarId: Int,
    val taxiCarManufacturer: String,
    val taxiCarModel: String,
    val taxiCarNumber: String,
    val taxiCarScheduleId: Int,
    val taxiCreatedAt: String,
    val taxiStartLat: Double,
    val taxiStartLon: Double,
    val taxiEndLat: Double,
    val taxiEndLon: Double,
    val taxiStartTime: String,
    val taxiEndTime: String,
    val taxiCarImg: String,
    val taxiPrice: Int,
    val taxiTime: Int,
    val taxiStatus: String,
    val startAddress: String,
    val endAddress: String,
)
