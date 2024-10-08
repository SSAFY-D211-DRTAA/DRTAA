package com.drtaa.core_model.util

import com.drtaa.core_model.network.ResponseRentCarCall
import com.drtaa.core_model.rent.CarPosition

fun ResponseRentCarCall.toCarInfo(): CarPosition {
    return CarPosition(
        latitude = rentCarLat,
        longitude = rentCarLon,
        rentId = rentId.toInt(),
        travelId = travelId.toInt(),
        travelDatesId = travelDatesId.toInt(),
        datePlacesId = datePlacesId.toInt()
    )
}