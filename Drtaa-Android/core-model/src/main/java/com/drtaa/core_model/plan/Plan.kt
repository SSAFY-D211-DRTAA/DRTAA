package com.drtaa.core_model.plan

data class Plan(
    val datesDetail: List<DayPlan>,
    val travelEndDate: String,
    val travelName: String,
    val travelStartDate: String
) {
    data class DayPlan(
        val placesDetail: List<PlanItem>,
        val travelDatesDate: String,
        val travelDatesId: Int
    ) {
        data class PlanItem(
            val datePlacesAddress: String,
            val datePlacesCategory: String,
            val datePlacesIsVisited: Boolean,
            val datePlacesLat: Double,
            val datePlacesLon: Double,
            val datePlacesName: String,
            val datePlacesId: Int
        )
    }
}