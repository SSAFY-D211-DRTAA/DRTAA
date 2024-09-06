package com.drtaa.feature_main.util

object Constant {
    enum class Page(val hideBottomNav: Boolean) {
        MAP(false),
        MY_PAGE(false)
        ;

        companion object {
            fun fromId(id: Int): Page? = entries.find { it.id == id }
        }

        val id: Int
            get() = when (this) {
                MAP -> com.drtaa.feature_ticket.R.id.mapFragment
                MY_PAGE -> com.drtaa.feature_mypage.R.id.myPageFragment
            }
    }
}
