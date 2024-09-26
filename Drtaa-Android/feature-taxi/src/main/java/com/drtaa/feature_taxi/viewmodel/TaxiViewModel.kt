package com.drtaa.feature_taxi.viewmodel

import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.TaxiRepository
import com.drtaa.core_model.map.Search
import com.naver.maps.geometry.LatLng
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class TaxiViewModel @Inject constructor(
    private val taxiRepository: TaxiRepository
):ViewModel(){

    private val _taxiStartLocation = MutableStateFlow<Search?>(null)
    val taxiStartLocation: StateFlow<Search?> = _taxiStartLocation

    private val _taxiEndLocation = MutableStateFlow<Search?>(null)
    val taxiEndLocation: StateFlow<Search?> = _taxiEndLocation


    fun setTaxiStartLocation(search: Search){
        viewModelScope.launch {
            val taxiStart = Search(
                title = search.title,
                category = search.category,
                roadAddress = search.roadAddress,
                lng = default_position.longitude,
                lat = default_position.latitude
            )

            _taxiStartLocation.value = taxiStart
        }
    }

    fun setTaxiEndLocation(search: Search){
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


    companion object {
        val default_position = LatLng(37.57578754990568, 126.90027478459672)
    }
}