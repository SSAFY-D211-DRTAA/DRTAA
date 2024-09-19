package com.drtaa.core_data.datasource

import com.drtaa.core_model.network.ResponseTour

interface TourDataSource {
    suspend fun getLocationBasedList(
        numOfRows: Int,
        pageNo: Int,
        mapX: String,
        mapY: String,
        radius: String,
    ): ResponseTour
}