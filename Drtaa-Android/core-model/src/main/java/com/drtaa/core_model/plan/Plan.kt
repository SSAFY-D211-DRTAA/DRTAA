package com.drtaa.core_model.plan

data class Plan(
    val travelId: Int,
    val datesDetail: List<DayPlan>,
    val travelEndDate: String,
    val travelName: String,
    val travelStartDate: String
) {
    data class DayPlan(
        val travelId: Int,
        val placesDetail: List<PlanItem>,
        val travelDatesDate: String,
        val travelDatesId: Int
    ) {
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
        )
    }
}