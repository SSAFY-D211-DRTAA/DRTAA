package com.drtaa.core_model.travel

data class Weather(
    val dayOfMonth: Int,
    val dayOfWeek: String,
    val description: String,
    val feelsLike: Double,
    val humidity: Int,
    val max: Double,
    val min: Double,
    val pop: Double
)