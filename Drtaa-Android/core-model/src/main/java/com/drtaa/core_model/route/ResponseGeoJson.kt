package com.drtaa.core_model.route

data class ResponseGeoJson(
    val type: String,
    val features: List<Feature>
)

data class Feature(
    val type: String,
    val geometry: Geometry,
    val properties: Properties
)

data class Geometry(
    val type: String,
    val coordinates: List<Any>
)

data class Properties(
    val totalDistance: Int?,
    val totalTime: Int?,
    val totalFare: Int?,
    val taxiFare: Int?,
    val index: Int?,
    val pointIndex: Int?,
    val name: String?,
    val description: String?,
    val nextRoadName: String?,
    val turnType: Int?,
    val pointType: String?,
    val lineIndex: Int?,
    val distance: Int?,
    val time: Int?,
    val roadType: Int?,
    val facilityType: Int?
)