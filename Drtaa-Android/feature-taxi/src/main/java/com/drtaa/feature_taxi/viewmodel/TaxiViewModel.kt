package com.drtaa.feature_taxi.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.TaxiRepository
import com.drtaa.core_model.map.Search
import com.drtaa.core_model.route.ResponseGeoJson
import com.drtaa.core_model.route.RouteInfo
import com.naver.maps.geometry.LatLng
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class TaxiViewModel @Inject constructor(
    private val taxiRepository: TaxiRepository
) : ViewModel() {

    private val _taxiStartLocation = MutableStateFlow<Search?>(null)
    val taxiStartLocation: StateFlow<Search?> = _taxiStartLocation

    private val _taxiEndLocation = MutableStateFlow<Search?>(null)
    val taxiEndLocation: StateFlow<Search?> = _taxiEndLocation

    private val _routeOverlay = MutableStateFlow<List<LatLng>?>(null)
    val routeOverlay: StateFlow<List<LatLng>?> = _routeOverlay.asStateFlow()

    private val _routeInfo = MutableStateFlow<RouteInfo?>(null)
    val routeInfo: StateFlow<RouteInfo?> = _routeInfo.asStateFlow()

    fun setTaxiStartLocation(search: Search) {
        viewModelScope.launch {
            val taxiStart = Search(
                title = search.title,
                category = search.category,
                roadAddress = search.roadAddress,
                lng = search.lng,
                lat = search.lat
            )

            _taxiStartLocation.value = taxiStart
        }
    }

    fun setTaxiEndLocation(search: Search) {
        viewModelScope.launch {
            val taxiEnd = Search(
                title = search.title,
                category = search.category,
                roadAddress = search.roadAddress,
                lng = search.lng,
                lat = search.lat
            )
            _taxiEndLocation.value = taxiEnd
        }
    }
    
    fun getRoute(start: Search, end: Search) {
        viewModelScope.launch {
            taxiRepository.getRoute(start, end).collect { result ->
                result.onSuccess { geoJson ->

                    val route = parseGeoJons(geoJson)
                    _routeOverlay.value = route

                    geoJson.features.firstOrNull()?.properties.let { properties ->
                        _routeInfo.value = RouteInfo(
                            totalDistance = properties?.totalDistance ?: 0,
                            totalTime = properties?.totalTime ?: 0,
                            totalFare = properties?.totalFare ?: 0,
                            taxiFare = properties?.taxiFare ?: 0
                        )
                    }
                    Timber.d("경로 가져오기 성공")
                }.onFailure {
                    Timber.d("경로 가져오기 에러")
                }
            }
        }
    }

    private fun parseGeoJons(geoJson: ResponseGeoJson): List<LatLng> {
        return geoJson.features.filter {
            it.geometry.type == "LineString"
        }.flatMap { feature ->
            (feature.geometry.coordinates as List<List<Double>>).map { coordinate ->
                LatLng(coordinate[1], coordinate[0])
            }
        }
    }
}