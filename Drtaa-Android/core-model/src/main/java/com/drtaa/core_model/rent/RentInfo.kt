package com.drtaa.core_model.rent

import com.drtaa.core_model.map.Search

data class RentInfo(
    val carInfo: RentCar?,
    val hours: Double,
    val price: Int,
    val discount: Int,
    val finalPrice: Int,
    val startLocation: Search,
    val startSchedule: RentSchedule,
    val endSchedule: RentSchedule,
    val people: Int
)
