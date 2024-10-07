package com.drtaa.core_model.util

const val ONE_HOUR_IN_MILLIS = 3600000

enum class Social(val type: String) {
    NAVER("Naver"),
    GOOGLE("Goolge")
}

enum class Time(val value: Int) {
    HOUR_TO_MILLIS(ONE_HOUR_IN_MILLIS),
}

enum class Pay(val type: String) {
    SUCCESS("success"),
    CLOSED("closed"),
    CANCELED("canceled")
}

enum class Status(val status: String) {
    IN_PROGRESS("in_progress"),
    RESERVED("reserved"),
    COMPLETED("completed"),
    IDLING("idling"),
    CALLING("calling"),
    DRIVING("driving"),
    PARKING("parking"),
    WAITING("waiting"),
    CHARGING("charging")
}