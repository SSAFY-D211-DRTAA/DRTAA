package com.drtaa.feature_plan.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.TravelRepository
import com.drtaa.core_model.travel.NaverPost
import com.drtaa.core_model.travel.Weather
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class TravelViewModel @Inject constructor(
    private val travelRepository: TravelRepository
) : ViewModel() {

    private val _postList = MutableStateFlow<List<NaverPost>?>(null)
    val postList: StateFlow<List<NaverPost>?> = _postList

    private val _weatherList = MutableStateFlow<List<Weather>?>(null)
    val weatherList: StateFlow<List<Weather>?> = _weatherList

    fun getBlogPostList(keyword: String) {
        viewModelScope.launch {
            travelRepository.getBlogPostList(keyword).collect { result ->
                result.onSuccess { data ->
                    _postList.value = data
                }.onFailure {
                    _postList.value = null
                }
            }
        }
    }

    fun getWeatherList(lat: Double, lon: Double) {
        viewModelScope.launch {
            travelRepository.getWeatherList(lat, lon).collect { result ->
                result.onSuccess { data ->
                    _weatherList.value = data
                }.onFailure {
                    _weatherList.value = null
                }
            }
        }
    }
}
