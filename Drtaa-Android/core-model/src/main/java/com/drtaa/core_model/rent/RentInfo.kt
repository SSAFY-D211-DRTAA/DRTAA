package com.drtaa.core_model.rent

data class RentInfo(
    val isAvailable: Boolean,
    val carName: String,
    val carImg: String,
    val fareType: String,
    val fareCount: Int,
    val price: String,
    val discount: String,
    val totalPrice: String,
    val startLocation: String,
    val startDate: String,
    val endDate: String,
    val startTime: String,
    val endTime: String,
    val people: Int
)
