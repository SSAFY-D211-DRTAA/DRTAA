package com.drtaa.core_model.rent

import com.drtaa.core_model.map.Search

data class RentInfo(
    val carInfo: RentCar?,
    val isHour: Boolean,
    val fareCount: Int,
    val price: Int,
    val discount: Int,
    val totalPrice: Int = price - discount,
    val startLocation: Search,
    val startSchedule: RentSchedule,
    val endSchedule: RentSchedule,
    val people: Int
)
