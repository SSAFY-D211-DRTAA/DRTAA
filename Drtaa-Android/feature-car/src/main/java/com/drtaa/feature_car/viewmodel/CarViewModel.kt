package com.drtaa.feature_car.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.GPSRepository
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_model.network.RequestCompleteRent
import com.naver.maps.geometry.LatLng
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class CarViewModel @Inject constructor(
    private val gpsRepository: GPSRepository,
    private val rentRepository: RentRepository
) : ViewModel() {
    private var publishJob: Job? = null
    private val mqttScope = CoroutineScope(Dispatchers.IO)
    private val _gpsData = MutableSharedFlow<LatLng>()
    val gpsData = _gpsData.asSharedFlow()
    private val _trackingState = MutableStateFlow<Boolean>(false)
    val trackingState: StateFlow<Boolean> get() = _trackingState

    private val _isSuccessComplete = MutableSharedFlow<Boolean>()
    val isSuccessComplete: SharedFlow<Boolean> = _isSuccessComplete

    init {
        initMQTT()
        observeMqttMessages()
    }

    private fun initMQTT() {
        viewModelScope.launch {
            gpsRepository.setupMqttConnection()
        }
    }

    fun toggleTrackingState() {
        _trackingState.value = !_trackingState.value
    }

    fun startPublish(data: String = "GPS", intervalMillis: Long = DEFAULT_INTERVAL) {
        publishJob = mqttScope.launch {
            while (isActive) {
                gpsRepository.publishGpsData(data)
                delay(intervalMillis)
            }
        }
        gpsRepository.publishGpsData(data)
    }

    fun stopPublish() {
        publishJob?.cancel()
        publishJob = null
    }

    private fun observeMqttMessages() {
        viewModelScope.launch {
            gpsRepository.observeMqttMessages().collectLatest {
                Timber.tag("mqtt_viewmodel").d("observeMqttMessages: $it")
                _gpsData.emit(LatLng(it.latitude, it.longitude))
            }
        }
    }

    fun completeRent(rentId: Int, rentCarScheduleId: Int) {
        viewModelScope.launch {
            val requestCompleteRent = RequestCompleteRent(
                rentId = rentId,
                rentCarScheduleId = rentCarScheduleId
            )

            rentRepository.completeRent(requestCompleteRent).collect { result ->
                result.onSuccess {
                    Timber.d("성공")
                    _isSuccessComplete.emit(true)
                }.onFailure {
                    Timber.d("실패")
                    _isSuccessComplete.emit(false)
                }
            }
        }
    }

    companion object {
        private const val DEFAULT_INTERVAL = 2000L
    }
}