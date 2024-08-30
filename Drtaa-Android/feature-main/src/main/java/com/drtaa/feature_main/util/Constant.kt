package com.drtaa.feature_main.util

enum class Page(val hideBottomNav: Boolean) {
    MAP(false),
    MY_PAGE(false)
    ;

    companion object {
        fun fromId(id: Int): Page? = entries.find { it.id == id }
    }

    val id: Int
        get() = when (this) {
            MAP -> com.drtaa.feature_map.R.id.mapFragment
            MY_PAGE -> com.drtaa.feature_mypage.R.id.myPageFragment
        }
}