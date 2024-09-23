package com.drtaa.feature_main.util

import com.drtaa.feature_home.R

enum class Page(val hideBottomNav: Boolean) {
    HOME(false),
    CAR(false),
    TOUR(false),
    MY_PAGE(false),
    RENT(true),
    RENT_SEARCH(true),
    RENT_PLAN(true),
    RENT_SUMMARY(true),
    PLAN_LIST(true),
    PLAN_SEARCH(true),
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
            RENT_PLAN -> com.drtaa.feature_rent.R.id.rentPlanFragment
            RENT_SUMMARY -> com.drtaa.feature_rent.R.id.rentSummaryFragment
            PLAN_LIST -> com.drtaa.feature_plan.R.id.planListFragment
            PLAN_SEARCH -> com.drtaa.feature_plan.R.id.planSearchFragment
        }
}