package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.BuildConfig
import com.drtaa.core_data.datasource.TourDataSource
import com.drtaa.core_model.network.ResponseTour
import com.drtaa.core_network.api.TourAPI
import timber.log.Timber
import javax.inject.Inject

class TourDataSourceImpl @Inject constructor(private val api: TourAPI) : TourDataSource {
    override suspend fun getLocationBasedList(
        numOfRows: Int,
        pageNo: Int,
        mapX: String,
        mapY: String,
        radius: String,
    ): ResponseTour {
        Timber.tag("tour").d("tour datasource call")
        return api.getLocationBasedList(
            numOfRows = numOfRows,
            pageNo = pageNo,
            mobileOS = "AND",
            mobileApp = "Drtaa",
            type = "json",
            mapX = mapX,
            mapY = mapY,
            radius = radius,
            serviceKey = BuildConfig.TOUR_API_KEY
        )
    }
}