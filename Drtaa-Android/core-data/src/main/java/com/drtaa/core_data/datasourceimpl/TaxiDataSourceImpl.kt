package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.TaxiDataSource
import com.drtaa.core_model.map.Search
import com.drtaa.core_model.route.ResponseGeoJson
import com.drtaa.core_model.taxi.RequestCallTaxi
import com.drtaa.core_model.taxi.TaxiDetail
import com.drtaa.core_network.api.TaxiAPI
import com.drtaa.core_network.api.TmapAPI
import javax.inject.Inject

class TaxiDataSourceImpl @Inject constructor(
    private val tmapAPI: TmapAPI,
    private val taxiAPI: TaxiAPI
) : TaxiDataSource {
    override suspend fun getRoute(start: Search, end: Search): ResponseGeoJson {
        return tmapAPI.getRoute(startX = start.lng, startY = start.lat, endX = end.lng, endY = end.lat)
    }

    override suspend fun callTaxi(requestCallTaxi: RequestCallTaxi): TaxiDetail {
        return taxiAPI.callTaxi(requestCallTaxi)
    }
}