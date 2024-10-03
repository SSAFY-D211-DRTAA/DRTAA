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

    // MQTT
    private var publishJob: Job? = null
    private val mqttScope = CoroutineScope(Dispatchers.IO)
    private val _gpsData = MutableSharedFlow<LatLng>()
    val gpsData = _gpsData.asSharedFlow()
    private val _mqttConnectionStatus = MutableStateFlow<Int>(-1)
    val mqttConnectionStatus: StateFlow<Int> = _mqttConnectionStatus
    private val _routeData = MutableStateFlow<List<CarRoute>>(emptyList())
    val routeData: StateFlow<List<CarRoute>> = _routeData
    private val _carPosition = MutableSharedFlow<CarPosition>()
    val carPosition: SharedFlow<CarPosition> = _carPosition.asSharedFlow()

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

    // CAR TRACKING FRAGMENT
    private val _isReturn = MutableStateFlow<Boolean>(false)
    val isReturn: StateFlow<Boolean> = _isReturn
    private val _isFirst = MutableSharedFlow<Boolean>()
    val isFirst: SharedFlow<Boolean> = _isFirst.asSharedFlow()
    private val _isRecall = MutableSharedFlow<Boolean>()
    val isRecall: SharedFlow<Boolean> = _isRecall.asSharedFlow()
    private val _drivingStatus = MutableStateFlow<CarStatus>(CarStatus.IDLE)
    val drivingStatus: StateFlow<CarStatus> = _drivingStatus
    private val _trackingState = MutableStateFlow<Boolean>(false)
    val trackingState: StateFlow<Boolean> get() = _trackingState

    init {
        initMQTT()
        getRentTravelInfo()
        getCarDrivingStatus()
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
        publishJob?.cancel()
        publishJob = null
    }

    private fun observeMqttMessages() {
        viewModelScope.launch {
            gpsRepository.observeMqttGPSMessages().collectLatest {
                Timber.tag("Car").d("observeMqttMessages: $it")
                val lat = it.msg.latitude
                val lng = it.msg.longitude
                Timber.tag("Car").d("check: $lat, $lng")
                if(lat != null && lng != null){
                    _gpsData.emit(LatLng(lat, lng))
                }else{
                    Timber.tag("Car").d("null들어왔슈 $it")
                }
            }
        }
    }

    fun clearMqttStatus() {
        _mqttConnectionStatus.value = -1
    }

    fun getCarDrivingStatus() {
        viewModelScope.launch {
            _currentRentDetail.value?.let {
                rentCarRepository.getDriveStatus(it.rentId!!).collect { status ->
                    status.onSuccess { data ->
                        Timber.tag("Car").d("드라이브 상태 조회 성공 $data")
                        _drivingStatus.value = if (data == "driving") {
                            CarStatus.DRIVING
                        } else {
                            CarStatus.IDLE
                        }
                    }.onFailure { e ->
                        Timber.tag("Car").d("드라이브 상태 조회 실패 $e")
                        _drivingStatus.value = CarStatus.IDLE
                    }
                }
            }
        }
    }

    fun getOnCar(rentId: Long) {
        viewModelScope.launch {
            Timber.tag("Car").d("${_rentTravelInfo.value}")
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
                        Timber.tag("Car").d("탑승 $data")
                        rentCarRepository.setCarWithTravelInfo(
                            RequestCarStatus(
                                rentId = rentId.toInt(),
                                travelId = data.travelId,
                                travelDatesId = data.travelDatesId,
                                datePlacesId = data.datePlacesId
                            )
                        )
                        getCarDrivingStatus()
                    }.onFailure { e ->
                        Timber.tag("Car").d("승차 $e")
                    }
                }
            }
        }
    }

    fun getOffCar(rentId: Long) {
        viewModelScope.launch {
            _rentTravelInfo.value?.let {
                val reqeust = RequestCarStatus(
                    rentId = rentId.toInt(),
                    travelId = it.travelId,
                    travelDatesId = it.travelDatesId,
                    datePlacesId = it.datePlacesId
                )
                Timber.tag("Car").d("하차 reqeust: $reqeust")
                rentCarRepository.getOffCar(reqeust).collect { result ->
                    result.onSuccess { data ->
                        // 하차에 성공했다면 다음 목적지가 나올 것
                        Timber.tag("Car").d("하차성공 $data")
                        getCarDrivingStatus()
                        rentCarRepository.setCarWithTravelInfo(
                            RequestCarStatus(
                                rentId = rentId.toInt(),
                                travelId = data.travelId,
                                travelDatesId = data.travelDatesId,
                                datePlacesId = data.datePlacesId
                            )
                        )
                        Timber.tag("Car").d("하차 $data")
                    }.onFailure { e ->
                        Timber.tag("Car").d("하차 실패 $e")
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
                    Timber.tag("Car").d("여행 정보 조회 성공 $data")
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
                    getCarDrivingStatus()
                }.onFailure { e ->
                    // 진행중인 렌트 차량이 없을 때
                    _currentRentDetail.value = null
                    Timber.tag("Car").d("$e")
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
                    Timber.tag("Car").d("$e")
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

    /**
     *  반납
     */
    fun returnRent() {
        viewModelScope.launch {
            _currentRentDetail.value?.let {
                val requestCompleteRent = RequestCompleteRent(
                    rentId = it.rentId ?: -1,
                    rentCarScheduleId = it.rentCarScheduleId
                )
                rentRepository.completeRent(requestCompleteRent).collect { result ->
                    result.onSuccess {
                        Timber.tag("Car").d("성공")
                        stopPublish()
                        _currentRentDetail.value = null
                        _isReturn.value = true
                    }.onFailure {
                        Timber.tag("Car").d("실패")
                        _isReturn.value = false
                    }
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
                        val request = RequestCarStatus(
                            rentId = rent.rentId,
                            travelId = rent.travelId,
                            travelDatesId = rent.travelDatesId,
                            datePlacesId = rent.datePlacesId
                        )
                        Timber.tag("Car").d("첫호출 $request")
                        rentCarRepository.setCarWithTravelInfo(request)
                        _isFirst.emit(true)
                        Timber.tag("Car").d("성공 $rent")
                    }.onFailure {
                        Timber.tag("Car").d("실패")
                    }
                }
        }
    }

    /**
     * 재호출
     */
    fun recallAssignedCar(userPosition: LatLng) {
        _currentRentDetail.value?.rentId?.let { rentId ->
            _rentTravelInfo.value?.let { rentTravelInfo ->
                viewModelScope.launch {
                    rentCarRepository.callAssignedCar(
                        rentId,
                        userPosition.latitude,
                        userPosition.longitude,
                        rentTravelInfo.travelId.toLong(),
                        rentTravelInfo.travelDatesId.toLong(),
                        rentTravelInfo.datePlacesId.toLong()
                    ).collect { result ->
                        result.onSuccess {
                            Timber.tag("Car").d("재호출 성공 $it")
                            _isRecall.emit(true)
                            _carPosition.emit(it)
                        }.onFailure {
                            _isRecall.emit(false)
                            Timber.tag("Car").d("실패 $it")
                        }
                    }
                }
            } ?: Timber.tag("Car").d("rentTravelInfo가 null입니다.")
        } ?: Timber.tag("Car").d("rentId가 null입니다.")
    }

    fun getRoute() {
        viewModelScope.launch {
            gpsRepository.observeMqttPathMessages().collect { path ->
                Timber.tag("Car").d("$path")
                _routeData.value = path
            }
        }
    }

    companion object {
        private const val DEFAULT_INTERVAL = 1000L
    }
}