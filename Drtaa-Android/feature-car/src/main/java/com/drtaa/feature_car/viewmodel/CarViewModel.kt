package com.drtaa.feature_car.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.GPSRepository
import com.drtaa.core_data.repository.RentCarRepository
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_model.map.CarRoute
import com.drtaa.core_model.network.RequestCarStatus
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.ResponseRentStateAll
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

    private val _isSuccessComplete = MutableSharedFlow<Boolean>()
    val isSuccessComplete: SharedFlow<Boolean> = _isSuccessComplete

    private val _carPosition = MutableSharedFlow<CarPosition>()
    val carPosition: SharedFlow<CarPosition> = _carPosition.asSharedFlow()

    private val _latestReservedId = MutableStateFlow<Long>(0L)
    val latestReservedId: StateFlow<Long> = _latestReservedId

    private val _firstCall = MutableSharedFlow<Boolean>()
    val firstCall: SharedFlow<Boolean> = _firstCall.asSharedFlow()

    private val _drivingStatus = MutableStateFlow<CarStatus>(CarStatus.IDLE)
    val drivingStatus: StateFlow<CarStatus> = _drivingStatus

    private val _mqttConnectionStatus = MutableStateFlow<Int>(-1)
    val mqttConnectionStatus: StateFlow<Int> = _mqttConnectionStatus

    private val _routeData = MutableStateFlow<List<CarRoute>>(emptyList())
    val routeData: StateFlow<List<CarRoute>> = _routeData

    // CAR FRAGMENT
    private val _isInProgress = MutableStateFlow<Boolean>(false)
    val isInProgress: StateFlow<Boolean> = _isInProgress
    private val _isEmtpty = MutableStateFlow<Boolean>(false)
    val isEmpty: StateFlow<Boolean> = _isEmtpty
    private val _isReserved = MutableStateFlow<Boolean>(false)
    val isReserved: StateFlow<Boolean> = _isReserved
    private val _currentRentDetail = MutableStateFlow<RentDetail?>(null)
    val currentRentDetail: StateFlow<RentDetail?> = _currentRentDetail
    private val _reservationRent = MutableStateFlow<ResponseRentStateAll?>(null)
    val reservationRent: StateFlow<ResponseRentStateAll?> = _reservationRent
    private val _rentTravelInfo = MutableStateFlow<RequestCarStatus?>(null)
    val rentTravelInfo: StateFlow<RequestCarStatus?> = _rentTravelInfo

    // CAR DRIVING
    private val _carStatus = MutableStateFlow<CarStatus>(CarStatus.IDLE)
    val carStatus: StateFlow<CarStatus> = _carStatus


    init {
        initMQTT()
        getRentTravelInfo()
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

    fun getOnCar(rentId: Long) {
        viewModelScope.launch {
            Timber.tag("qr").d("${_rentTravelInfo.value}")
            _rentTravelInfo.value?.let {
                rentCarRepository.getOnCar(
                    RequestCarStatus(
                        rentId = rentId.toInt(),
                        travelId = it.travelId,
                        travelDatesId = it.travelDatesId,
                        datePlacesId = it.datePlacesId
                    )
                ).collect { result ->
                    result.onSuccess { data ->
                        // 탑승에 성공했다면 update
                        Timber.tag("rent").d("탑승 $data")
                        _carStatus.value = CarStatus.DRIVING
                        rentCarRepository.setCarWithTravelInfo(
                            RequestCarStatus(
                                rentId = rentId.toInt(),
                                travelId = data.travelId,
                                travelDatesId = data.travelDatesId,
                                datePlacesId = data.datePlacesId
                            )
                        )
                    }.onFailure {
                        Timber.tag("rent").d("현재 진행 중인 렌트가 없습니다.")
                    }
                }
            }
        }
    }

    fun getOffCar(rentId: Long) {
        viewModelScope.launch {
            _rentTravelInfo.value?.let {
                rentCarRepository.getOffCar(
                    RequestCarStatus(
                        rentId = rentId.toInt(),
                        travelId = it.travelId,
                        travelDatesId = it.travelDatesId,
                        datePlacesId = it.datePlacesId
                    )
                ).collect { result ->
                    result.onSuccess { data ->
                        _carStatus.value = CarStatus.PARKING
                        // 하차에 성공했다면 다음 목적지가 나올 것
                        Timber.tag("rent").d("하차 $data")
                    }.onFailure {
                        Timber.tag("rent").d("현재 진행 중인 렌트가 없습니다.")
                    }
                }
            }
        }
    }

    fun getRentTravelInfo() {
        viewModelScope.launch {
            rentCarRepository.getCarWithTravelInfo().collect {
                it.onSuccess { data ->
                    _rentTravelInfo.value = data
                }
            }
        }
    }

    /**
     *  진행 중인 렌트차량이 있으면 그걸 가져오고, 없다면 예약한 차량을 가져온다. 그거도 없으면 예약 없음
     */
    fun getValidRent() {
        viewModelScope.launch {
            rentRepository.getCurrentRent().collect { inProgress ->
                inProgress.onSuccess { activeCar ->
                    _isInProgress.value = true
                    _currentRentDetail.value = activeCar
                }.onFailure { e ->
                    // 진행중인 렌트 차량이 없을 때
                    _currentRentDetail.value = null
                    Timber.tag("current rent").d("$e")
                    _isInProgress.value = false
                    getReservedRent()
                }
            }
        }
    }

    private fun getReservedRent() {
        viewModelScope.launch {
            rentRepository.getReservedRentState().collect { reserved ->
                reserved.onSuccess {
                    // 예약한 차량이 있을 때
                    _isReserved.value = true
                    _reservationRent.value = it
                }.onFailure { e ->
                    Timber.tag("current rent").d("$e")
                    _reservationRent.value = null
                    _isReserved.value = false
                    checkEmpty()
                }
            }
        }
    }

    private fun checkEmpty() {
        _isEmtpty.value = !(_isReserved.value || _isInProgress.value)
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

    /**
     *  반납
     */
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
                    _carStatus.value = CarStatus.IDLE
                    _currentRentDetail.value = null
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
            rentCarRepository.callAssignedCar(
                _currentRentDetail.value?.rentId!!,
                userPosition.latitude,
                userPosition.longitude,
                _rentTravelInfo.value!!.travelId.toLong(),
                _rentTravelInfo.value!!.travelDatesId.toLong(),
                _rentTravelInfo.value!!.datePlacesId.toLong()
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

    /**
     *  첫 호출
     */
    fun callFirstAssignedCar() {
        viewModelScope.launch {
            rentCarRepository.callFirstAssignedCar(_reservationRent.value!!.rentId)
                .collect { result ->
                    result.onSuccess { rent ->
                        rentCarRepository.setCarWithTravelInfo(
                            rentInfo = RequestCarStatus(
                                rentId = rent.rentId,
                                travelId = rent.travelId,
                                travelDatesId = rent.travelDatesId,
                                datePlacesId = rent.travelDatesId
                            )
                        )
                        _firstCall.emit(true)
                        Timber.tag("first call").d("성공 $rent")
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