package com.drtaa.core_model.taxi

import com.drtaa.core_model.map.Search
import com.drtaa.core_model.rent.RentCar
import com.drtaa.core_model.rent.RentSchedule

data class TaxiInfo(
    val carInfo: RentCar?,
    val minutes: Int,
    val price: Int,
    val startLocation: Search,
    val endLocation: Search,
    val startSchedule: RentSchedule,
    val endSchedule: RentSchedule
)
