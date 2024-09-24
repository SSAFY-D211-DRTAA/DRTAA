package com.drtaa.feature_car.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.GPSRepository
import com.drtaa.core_data.repository.RentCarRepository
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.rent.CarPosition
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
    private val rentRepository: RentRepository,
    private val rentCarRepository: RentCarRepository,
) : ViewModel() {
    private var publishJob: Job? = null
    private val mqttScope = CoroutineScope(Dispatchers.IO)

    private val _gpsData = MutableSharedFlow<LatLng>()
    val gpsData = _gpsData.asSharedFlow()

    private val _trackingState = MutableStateFlow<Boolean>(false)
    val trackingState: StateFlow<Boolean> get() = _trackingState

    private val _currentRentDetail = MutableStateFlow<RentDetail?>(null)
    val currentRentDetail: StateFlow<RentDetail?> = _currentRentDetail

    private val _isSuccessComplete = MutableSharedFlow<Boolean>()
    val isSuccessComplete: SharedFlow<Boolean> = _isSuccessComplete

    private val _carPosition = MutableSharedFlow<CarPosition>()
    val carPosition: SharedFlow<CarPosition> = _carPosition.asSharedFlow()

    private val _latestReservedId = MutableStateFlow<Long>(0L)
    val latestReservedId: StateFlow<Long> = _latestReservedId

    init {
        getCurrentRent()
        getLatestRent()
        observeMqttMessages()
    }

    private fun getLatestRent() {
        viewModelScope.launch {
            rentRepository.getAllRentState().collect { result ->
                result.onSuccess { data ->
                    Timber.tag("rent latest").d("성공 $data")
                    _latestReservedId.value = data
                }.onFailure {
                    Timber.tag("rent").d("현재 진행 중인 렌트가 없습니다.")
                }
            }
        }
    }

    /**
     * 탑승처리 in-progress를 해줘야 렌트한 것으로 간주한다.
     */
    private fun getCurrentRent() {
        viewModelScope.launch {
            rentRepository.getCurrentRent().collect { result ->
                result.onSuccess { data ->
                    Timber.d("성공")
                    _currentRentDetail.value = data
                }.onFailure {
                    Timber.d("현재 진행 중인 렌트가 없습니다.")
                    _currentRentDetail.value = null
                }
            }
        }
    }

    fun initMQTT() {
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
        intervalMillis: Long = DEFAULT_INTERVAL,
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
            val rentDetail = getRentDetail() ?: return@launch
            Timber.tag("complete").d("$rentDetail")
            val requestCompleteRent = RequestCompleteRent(
                rentId = rentDetail.rentId ?: -1,
                rentCarScheduleId = rentDetail.rentCarScheduleId
            )

            rentRepository.completeRent(requestCompleteRent).collect { result ->
                result.onSuccess {
                    Timber.tag("complete").d("성공")
                    _isSuccessComplete.emit(true)
                }.onFailure {
                    Timber.tag("complete").d("실패")
                    _isSuccessComplete.emit(false)
                }
            }
        }
    }

    private suspend fun getRentDetail(): RentDetail? {
        return currentRentDetail.first()
    }

    fun callAssignedCar(userPosition: LatLng) {
        viewModelScope.launch {
            val rentId = _latestReservedId.value
            rentCarRepository.callAssignedCar(
                rentId,
                userPosition.latitude,
                userPosition.longitude
            ).collect { result ->
                result.onSuccess {
                    Timber.tag("call car").d("성공")
                    _carPosition.emit(it)
                }.onFailure {
                    Timber.tag("call car").d("실패")
                    _carPosition.emit(CarPosition(0.0, 0.0))
                }
            }
        }
    }

    companion object {
        private const val DEFAULT_INTERVAL = 2000L
    }
}