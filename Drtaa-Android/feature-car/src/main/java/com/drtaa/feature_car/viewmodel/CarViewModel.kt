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

    private val _rentState = MutableStateFlow<Boolean>(false)
    val rentState: StateFlow<Boolean> = _rentState

    private val _firstCall = MutableStateFlow<Boolean>(false)
    val firstCall: StateFlow<Boolean> = _firstCall

    init {
        getLatestRent()
        getCurrentRent()
        observeMqttMessages()
    }

    private fun getLatestRent() {
        viewModelScope.launch {
            rentRepository.getAllRentState().collect { result ->
                result.onSuccess { data ->
                    Timber.tag("rent latest").d("성공 $data")
                    _latestReservedId.value = data
                }.onFailure {
                    _latestReservedId.value = -1L
                    Timber.tag("rent").d("현재 진행 중인 렌트가 없습니다.")
                }
            }
        }
    }

    fun getOnCar(rentId: Long) {
        viewModelScope.launch {
            rentCarRepository.getOnCar(rentId).collect { result ->
                result.onSuccess { data ->
                    Timber.tag("rent latest").d("성공 $data")
                    _currentRentDetail.value?.let {
                        rentCarRepository.getDriveStatus(it.rentCarId.toLong()).collect { result ->
                            result.onSuccess { data ->
                                Timber.tag("rent latest").d("성공 $data")
                            }.onFailure {
                                Timber.tag("rent").d("현재 진행 중인 렌트가 없습니다.")
                            }
                        }
                    }

                    _rentState.value = true
                }.onFailure {
                    Timber.tag("rent").d("현재 진행 중인 렌트가 없습니다.")
                    _rentState.value = false
                }
            }
        }
    }

    fun getOffCar(rentId: Long) {
        viewModelScope.launch {
            rentCarRepository.getOffCar(rentId).collect { result ->
                result.onSuccess { data ->
                    Timber.tag("rent latest").d("성공 $data")
                    _rentState.value = false
                }.onFailure {
                    Timber.tag("rent").d("현재 진행 중인 렌트가 없습니다.")
                    _rentState.value = false
                }
            }
        }
    }

    /**
     * 탑승처리를 해줘야 렌트한 것으로 간주한다.
     */
    fun getCurrentRent() {
        viewModelScope.launch {
            rentRepository.getRentDetail(_latestReservedId.value).collect { result ->
                result.onSuccess { data ->
                    Timber.d("성공")
                    if (data.rentStatus == "in_progress") {
                        _currentRentDetail.value = data
                    }
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

    fun callFirstAssignedCar() {
        viewModelScope.launch {
            rentCarRepository.callFirstAssignedCar(_latestReservedId.value).collect { result ->
                result.onSuccess {
                    _firstCall.value = true
                    Timber.tag("첫 호출").d("성공")
                }.onFailure {
                    _firstCall.value = false
                    Timber.tag("첫 호출").d("실패")
                }
            }
        }
    }


    companion object {
        private const val DEFAULT_INTERVAL = 2000L
    }
}