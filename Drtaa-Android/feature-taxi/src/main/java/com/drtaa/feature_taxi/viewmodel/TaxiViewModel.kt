package com.drtaa.feature_taxi.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_data.repository.TaxiRepository
import com.drtaa.core_model.map.Search
import com.drtaa.core_model.network.RequestDuplicatedSchedule
import com.drtaa.core_model.rent.RentSchedule
import com.drtaa.core_model.route.ResponseGeoJson
import com.drtaa.core_model.route.RouteInfo
import com.drtaa.core_model.taxi.TaxiInfo
import com.naver.maps.geometry.LatLng
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
import java.time.format.TextStyle
import java.util.Locale
import javax.inject.Inject

@HiltViewModel
class TaxiViewModel @Inject constructor(
    private val taxiRepository: TaxiRepository,
    private val rentRepository: RentRepository
) : ViewModel() {

    private val _taxiStartLocation = MutableStateFlow<Search?>(null)
    val taxiStartLocation: StateFlow<Search?> = _taxiStartLocation

    private val _taxiEndLocation = MutableStateFlow<Search?>(null)
    val taxiEndLocation: StateFlow<Search?> = _taxiEndLocation

    private val _routeOverlay = MutableStateFlow<List<LatLng>?>(null)
    val routeOverlay: StateFlow<List<LatLng>?> = _routeOverlay.asStateFlow()

    private val _routeInfo = MutableStateFlow<RouteInfo?>(null)
    val routeInfo: StateFlow<RouteInfo?> = _routeInfo.asStateFlow()

    private val _isDuplicatedSchedule = MutableSharedFlow<Boolean?>()
    val isDuplicatedSchedule: SharedFlow<Boolean?> = _isDuplicatedSchedule

    private val _taxiInfo = MutableStateFlow<TaxiInfo?>(null)
    val taxiInfo: StateFlow<TaxiInfo?> = _taxiInfo

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

    fun getTaxiInfo() {
        viewModelScope.launch {
            val price = routeInfo.value?.taxiFare
            val now = LocalDateTime.now()
            val dayOfWeek = now.dayOfWeek.getDisplayName(TextStyle.SHORT, Locale.getDefault())
            _taxiInfo.emit(
                TaxiInfo(
                    carInfo = null,
                    minutes = routeInfo.value?.totalTime!!,
                    price = 100,
                    startLocation = _taxiStartLocation.value!!,
                    endLocation = _taxiEndLocation.value!!,
                    startSchedule = RentSchedule(
                        year = now.year,
                        month = now.monthValue,
                        date = now.dayOfMonth,
                        day = dayOfWeek,
                        hour = now.hour,
                        minute = now.minute
                    ),
                    endSchedule = RentSchedule(
                        year = now.year,
                        month = now.monthValue,
                        date = now.dayOfMonth,
                        day = dayOfWeek,
                        hour = now.hour,
                        minute = now.minute
                    )
                )
            )
        }
    }

    fun checkDuplicatedSchedule() {
        viewModelScope.launch {
            rentRepository.checkDuplicatedRent(
                RequestDuplicatedSchedule(
                    rentStartTime = LocalDateTime.now()
                        .format(DateTimeFormatter.ofPattern("yyyy-MM-dd")),
                    rentEndTime = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd"))
                )
            ).collect { result ->
                result.onSuccess { data ->
                    _isDuplicatedSchedule.emit(data)
                }.onFailure {
                    _isDuplicatedSchedule.emit(null)
                }
            }
        }
    }
}