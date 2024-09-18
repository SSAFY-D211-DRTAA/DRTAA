package com.drtaa.feature_main.util

enum class Page(val hideBottomNav: Boolean) {
    HOME(false),
    CAR(false),
    TOUR(false),
    MY_PAGE(false),
    RENT(true),
    RENT_SEARCH(true)
    ;

    companion object {
        fun fromId(id: Int): Page? = entries.find { it.id == id }
    }

    val id: Int
        get() = when (this) {
            HOME -> com.drtaa.feature_home.R.id.homeFragment
            CAR -> com.drtaa.feature_car.R.id.carFragment
            TOUR -> com.drtaa.feature_tour.R.id.tourFragment
            MY_PAGE -> com.drtaa.feature_mypage.R.id.myPageFragment
            RENT -> com.drtaa.feature_rent.R.id.rentFragment
            RENT_SEARCH -> com.drtaa.feature_rent.R.id.rentSearchFragment
        }
}