package com.drtaa.feature_car.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.GPSRepository
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.rent.RentDetail
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
import kotlinx.coroutines.flow.first
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

    private val _currentRentDetail = MutableSharedFlow<RentDetail?>()
    val currentRentDetail: SharedFlow<RentDetail?> = _currentRentDetail

    private val _isSuccessComplete = MutableSharedFlow<Boolean>()
    val isSuccessComplete: SharedFlow<Boolean> = _isSuccessComplete

    init {
        getCurrentRent()
        initMQTT()
        observeMqttMessages()
    }

    private fun getCurrentRent() {
        viewModelScope.launch {
            rentRepository.getCurrentRent().collect { result ->
                Timber.d("진행 중 렌트 가져오기 데이터 $result")
                result.onSuccess { data ->
                    Timber.d("성공")
                    _currentRentDetail.emit(data)
                }.onFailure {
                    Timber.d("현재 진행 중인 렌트가 없습니다.")
                    _currentRentDetail.emit(null)
                }
            }
        }
    }

    private fun initMQTT() {
        viewModelScope.launch {
            gpsRepository.setupMqttConnection()
        }
    }

    fun toggleTrackingState() {
        _trackingState.value = !_trackingState.value
    }

    fun startPublish(
        data: String =
            """
         {"action":"vehicle_gps"}
            """.trimIndent(),
        intervalMillis: Long = DEFAULT_INTERVAL
    ) {
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

    fun completeRent() {
        viewModelScope.launch {
            val rentDetail = currentRentDetail.first() ?: return@launch

            val requestCompleteRent = RequestCompleteRent(
                rentId = rentDetail.rentId,
                rentCarScheduleId = rentDetail.rentCarScheduleId
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