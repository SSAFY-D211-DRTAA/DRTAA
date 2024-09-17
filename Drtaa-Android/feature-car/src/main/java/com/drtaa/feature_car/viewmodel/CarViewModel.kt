package com.drtaa.feature_car.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.GPSRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class CarViewModel @Inject constructor(
    private val gpsRepository: GPSRepository
) : ViewModel() {
    private val _gpsData = MutableSharedFlow<String>()
    val gpsData = _gpsData.asSharedFlow()

    init {
        initMQTT()
        observeMqttMessages()
    }

    private fun initMQTT() {
        viewModelScope.launch {
            gpsRepository.setupMqttConnection()
        }
    }

    fun publish(data: String = "GPS") {
        gpsRepository.publishGpsData(data)
    }

    private fun observeMqttMessages() {
        viewModelScope.launch {
            gpsRepository.observeMqttMessages().collectLatest {
                _gpsData.emit(it)
            }
        }
    }
}