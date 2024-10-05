package com.drtaa.feature_main.util

import com.drtaa.feature_home.R

enum class Page(val hideBottomNav: Boolean) {
    HOME(false),
    CAR(false),
    TOUR(false),
    MY_PAGE(false),
    RENT(true),
    RENT_SEARCH(true),
    RENT_SUMMARY(true),
    RENT_HISTORY(true),
    RENT_HISTORY_SUMMARY(true),
    PLAN_HISTORY(true),
    PLAN_LIST(true),
    PLAN_SEARCH(true),
    TAXI(true),
    TAXI_SEARCH(true),
    TAXI_SUMMARY(true),
    CAR_TRACKING(true),
    TRAVEL(true),
    PAYMENT(true)
    ;

    companion object {
        fun fromId(id: Int): Page? = entries.find { it.id == id }
    }

    val id: Int
        get() = when (this) {
            HOME -> R.id.homeFragment
            CAR -> com.drtaa.feature_car.R.id.carFragment
            TOUR -> com.drtaa.feature_tour.R.id.tourFragment
            MY_PAGE -> com.drtaa.feature_mypage.R.id.myPageFragment
            RENT -> com.drtaa.feature_rent.R.id.rentFragment
            RENT_SEARCH -> com.drtaa.feature_rent.R.id.rentSearchFragment
            RENT_SUMMARY -> com.drtaa.feature_rent.R.id.rentSummaryFragment
            RENT_HISTORY -> R.id.rentHistoryFragment
            RENT_HISTORY_SUMMARY -> R.id.rentHistorySummaryFragment
            TAXI -> com.drtaa.feature_taxi.R.id.taxiFragment
            TAXI_SEARCH -> com.drtaa.feature_taxi.R.id.taxiSearchFragment
            TAXI_SUMMARY -> com.drtaa.feature_taxi.R.id.taxiSummaryFragment
            PLAN_LIST -> com.drtaa.feature_plan.R.id.planListFragment
            PLAN_SEARCH -> com.drtaa.feature_plan.R.id.planSearchFragment
            PLAN_HISTORY -> com.drtaa.feature_plan.R.id.planHistoryFragment
            CAR_TRACKING -> com.drtaa.feature_car.R.id.carTrackingFragment
            TRAVEL -> com.drtaa.feature_travel.R.id.travelFragment
            PAYMENT -> com.drtaa.feature_payment.R.id.paymentDoneFragment
        }
}