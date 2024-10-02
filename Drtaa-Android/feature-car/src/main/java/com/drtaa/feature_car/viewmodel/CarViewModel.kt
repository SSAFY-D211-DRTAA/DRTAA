package com.drtaa.feature_car.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.GPSRepository
import com.drtaa.core_data.repository.RentCarRepository
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_model.map.CarRoute
import com.drtaa.core_model.network.RequestCarStatus
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

enum class CarStatus {
    DRIVING,
    PARKING,
    IDLE
}

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

    private val _firstCall = MutableSharedFlow<Boolean>()
    val firstCall: SharedFlow<Boolean> = _firstCall.asSharedFlow()

    private val _drivingStatus = MutableStateFlow<CarStatus>(CarStatus.IDLE)
    val drivingStatus: StateFlow<CarStatus> = _drivingStatus

    private val _mqttConnectionStatus = MutableStateFlow<Int>(-1)
    val mqttConnectionStatus: StateFlow<Int> = _mqttConnectionStatus

    private val _routeData = MutableStateFlow<List<CarRoute>>(emptyList())
    val routeData: StateFlow<List<CarRoute>> = _routeData

    init {
        initMQTT()
        getLatestRent()
        observeMqttConnectionStatus()
        observeMqttMessages()
    }

    override fun onCleared() {
        super.onCleared()
        Timber.tag("mqtt_viewmodel").d("onCleared")
        disconnectMQTT()
        stopPublish()
    }

    private fun observeMqttConnectionStatus() {
        viewModelScope.launch {
            gpsRepository.observeConnectionStatus().collectLatest {
                Timber.tag("mqtt_viewmodel").d("observeMqttConnectionStatus: $it")
                _mqttConnectionStatus.value = it
            }
        }
    }

    fun clearMqttStatus() {
        _mqttConnectionStatus.value = -1
    }

    private suspend fun getCarInfo() = _carPosition.first()

    fun getOnCar(rentId: Long) {
        viewModelScope.launch {

            val lastCarPosition = getCarInfo()
            Timber.tag("qr").d("$lastCarPosition")
//            rentCarRepository.getOnCar(
//                RequestCarStatus(
//                    rentId = rentId,
//                    travelId = lastCarPosition.travelId,
//                    travelDatesId = lastCarPosition.travelDatesId,
//                    datePlacesId = lastCarPosition.datePlacesId
//                )
//            ).collect { result ->
//                result.onSuccess { data ->
//                    Timber.tag("rent qr").d("성공 $data")
//                    _currentRentDetail.value?.let {
//                        rentCarRepository.getDriveStatus(it.rentCarId.toLong()).collect { result ->
//                            result.onSuccess { data ->
//                                _drivingStatus.value = CarStatus.DRIVING
//                                Timber.tag("rent qr").d("성공 $data")
//                            }.onFailure {
//                                _drivingStatus.value = CarStatus.IDLE
//                                Timber.tag("rent").d("현재 진행 중인 렌트가 없습니다.")
//                            }
//                        }
//                    }
//
//                    _rentState.value = true
//                }.onFailure {
//                    Timber.tag("rent").d("현재 진행 중인 렌트가 없습니다.")
//                    _rentState.value = false
//                }
//            }
        }
    }

    fun getOffCar(rentId: Long) {
        viewModelScope.launch {
            val lastCarPosition = getCarInfo()
            rentCarRepository.getOffCar(
                RequestCarStatus(
                    rentId = rentId,
                    travelId = lastCarPosition.travelId,
                    travelDatesId = lastCarPosition.travelDatesId,
                    datePlacesId = lastCarPosition.datePlacesId
                )
            ).collect { result ->
                result.onSuccess { data ->
                    _drivingStatus.value = CarStatus.PARKING
                    Timber.tag("rent latest").d("성공 $data")
                    _rentState.value = false
                }.onFailure {
                    _drivingStatus.value = CarStatus.IDLE
                    Timber.tag("rent").d("현재 진행 중인 렌트가 없습니다.")
                    _rentState.value = false
                }
            }
        }
    }

    fun getLatestRent() {
        viewModelScope.launch {
            rentRepository.getAllRentState().collect { result ->
                result.onSuccess { data ->
                    Timber.tag("rent latest").d("성공 $data")
                    _latestReservedId.value = data
                    getCurrentRent()
                }.onFailure {
                    _latestReservedId.value = -1L
                    Timber.tag("rent").d("현재 진행 중인 렌트가 없습니다.")
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
                    Timber.tag("getCR").d("성공 $data")
                    when (data.rentStatus) {
                        "reserved" -> _currentRentDetail.value = data
                        "in_progress" -> _currentRentDetail.value = data
                    }
                }.onFailure {
                    Timber.d("현재 진행 중인 렌트가 없습니다.")
                    _currentRentDetail.value = null
                }
            }
        }
    }

    private fun initMQTT() {
        viewModelScope.launch {
            gpsRepository.setupMqttConnection()
        }
    }

    private fun disconnectMQTT() {
        gpsRepository.disconnectMqtt()
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
        toggleTrackingState()
        gpsRepository.publishGpsData(data)
    }

    fun stopPublish() {
        _trackingState.value = false
        publishJob?.cancel()
        publishJob = null
    }

    private fun observeMqttMessages() {
        viewModelScope.launch {
            gpsRepository.observeMqttGPSMessages().collectLatest {
                Timber.tag("mqtt_viewmodel").d("observeMqttMessages: $it")
                _gpsData.emit(LatLng(it.msg.latitude, it.msg.longitude))
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
                    _currentRentDetail.value = null
                    _rentState.value = false
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

    /**
     * 재호출
     */
    fun callAssignedCar(userPosition: LatLng) {
        viewModelScope.launch {
            val carInfo = getCarInfo()
            val rentId = _latestReservedId.value
            rentCarRepository.callAssignedCar(
                rentId,
                userPosition.latitude,
                userPosition.longitude,
                carInfo.travelId,
                carInfo.travelDatesId,
                carInfo.datePlacesId
            ).collect { result ->
                result.onSuccess {
                    Timber.tag("call car").d("성공 $it")
                    _carPosition.emit(it)
                }.onFailure {
                    Timber.tag("call car").d("실패 $it")
                }
            }
        }
    }

    fun callFirstAssignedCar() {
        viewModelScope.launch {
            rentCarRepository.callFirstAssignedCar(_latestReservedId.value).collect { result ->
                result.onSuccess {
                    _carPosition.emit(it)
                    _firstCall.emit(true)
                    Timber.tag("first call").d("성공 $it")
                }.onFailure {
                    _firstCall.emit(false)
                    Timber.tag("first call").d("실패")
                }
            }
        }
    }

    fun getRoute() {
        viewModelScope.launch {
            gpsRepository.observeMqttPathMessages().collect { path ->
                Timber.tag("path").d("$path")
                _routeData.value = path
            }
        }
    }

    companion object {
        private const val DEFAULT_INTERVAL = 2000L
    }
}