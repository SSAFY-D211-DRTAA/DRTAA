package com.drtaa.core_model.plan

data class LastPlan(
    val travelId: Int,
    val travelDatesId: Int,
    val datePlacesName: String,
    val datePlacesCategory: String,
    val datePlacesAddress: String,
    val datePlacesLat: Double,
    val datePlacesLon: Double
)