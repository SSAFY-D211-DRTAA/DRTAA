package com.drtaa.core_model.plan

import kotlinx.serialization.Serializable

@Serializable
data class PlanItem(
    val travelDatesId: Int,
    val datePlacesAddress: String,
    val datePlacesCategory: String,
    val datePlacesIsVisited: Boolean,
    val datePlacesLat: Double,
    val datePlacesLon: Double,
    val datePlacesName: String,
    val datePlacesId: Int = 0,
    val datePlacesOrder: Int,
    var isSelected: Boolean = false
) : java.io.Serializable
