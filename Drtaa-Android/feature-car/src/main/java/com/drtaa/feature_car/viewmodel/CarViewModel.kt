package com.drtaa.feature_car.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.GPSRepository
import com.drtaa.core_data.repository.NaverRepository
import com.drtaa.core_data.repository.PlanRepository
import com.drtaa.core_data.repository.RentCarRepository
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_model.map.CarRoute
import com.drtaa.core_model.network.RequestCarStatus
import com.drtaa.core_model.network.RequestCompleteRent
import com.drtaa.core_model.network.ResponseRentStateAll
import com.drtaa.core_model.network.ResponseReverseGeocode
import com.drtaa.core_model.plan.Plan
import com.drtaa.core_model.plan.PlanItem
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
    CALLING,
    IDLE
}

@HiltViewModel
class CarViewModel @Inject constructor(
    private val gpsRepository: GPSRepository,
    private val rentRepository: RentRepository,
    private val rentCarRepository: RentCarRepository,
    private val naverRepository: NaverRepository,
    private val planRepository: PlanRepository,
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
    private val _rentCompleteToday = MutableSharedFlow<Boolean>()
    val rentCompleteToday: SharedFlow<Boolean> = _rentCompleteToday.asSharedFlow()

    // CAR TRACKING FRAGMENT
    private val _isReturn = MutableStateFlow<Boolean>(false)
    val isReturn: StateFlow<Boolean> = _isReturn
    private val _isFirst = MutableSharedFlow<Boolean>()
    val isFirst: SharedFlow<Boolean> = _isFirst.asSharedFlow()
    private val _isRecall = MutableSharedFlow<Boolean>()
    val isRecall: SharedFlow<Boolean> = _isRecall.asSharedFlow()
    private val _drivingStatus = MutableStateFlow<CarStatus?>(null)
    val drivingStatus: StateFlow<CarStatus?> = _drivingStatus
    private val _trackingState = MutableStateFlow<Boolean>(false)
    val trackingState: StateFlow<Boolean> get() = _trackingState
    private val _destination = MutableStateFlow<LatLng?>(null)
    val destination: StateFlow<LatLng?> = _destination
    private val _reverseGeocode = MutableStateFlow<Result<String>?>(null)
    val reverseGeocode: StateFlow<Result<String>?> = _reverseGeocode

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
        stopGPSPublish()
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

    fun startGPSPublish(
        data: String =
            """
         {"action":"vehicle_gps"}
            """.trimIndent(),
    ) {
        publishJob = mqttScope.launch {
            while (isActive) {
                gpsRepository.publishGpsData(data, GPS_SUB)
                delay(DEFAULT_INTERVAL)
            }
        }
        toggleTrackingState()
        gpsRepository.publishGpsData(data, GPS_SUB)
    }

    fun fetchPath(
        data: String =
            """
         {"action":"vehicle_global_path"}
            """.trimIndent(),
    ) {
        viewModelScope.launch {
            gpsRepository.fetchPath(data, PATH_SUB)
        }
    }

    fun stopGPSPublish() {
        publishJob?.cancel()
        publishJob = null
    }

    private fun observeMqttMessages() {
        viewModelScope.launch {
            gpsRepository.observeMqttGPSMessages().collectLatest {
//                Timber.tag("Car").d("observeMqttMessages: $it")
                val lat = it.msg.latitude
                val lng = it.msg.longitude
                Timber.tag("Car").d("check: $lat, $lng")
                if (lat != null && lng != null) {
                    _gpsData.emit(LatLng(lat, lng))
                } else {
                    Timber.tag("Car").d("null들어왔슈 $it")
                }
            }
        }
    }

    fun clearMqttStatus() {
        _mqttConnectionStatus.value = -1
    }

    fun setDestination(latLng: LatLng) {
        _destination.value = latLng
        getReverseGeocode(latLng.latitude, latLng.longitude)
    }

    fun clearDestination() {
        _destination.value = null
    }

    fun getCarDrivingStatus() {
        viewModelScope.launch {
            _currentRentDetail.value?.let {
                rentCarRepository.getDriveStatus(it.rentCarId.toLong()).collect { status ->
                    status.onSuccess { data ->
                        Timber.tag("Car").d("드라이브 상태 조회 성공 $data")
                        _drivingStatus.value = when (data) {
                            "driving" -> {
                                CarStatus.DRIVING
                            }

                            "calling" -> {
                                CarStatus.CALLING
                            }

                            "parking" -> {
                                CarStatus.DRIVING
                            }

                            else -> {
                                CarStatus.IDLE
                            }
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
                        val request = RequestCarStatus(
                            rentId = rentId.toInt(),
                            travelId = data.travelId,
                            travelDatesId = data.travelDatesId,
                            datePlacesId = data.datePlacesId
                        )
                        Timber.tag("Car 탑승").d("$request")
                        rentCarRepository.setCarWithTravelInfo(
                            request
                        )
                        getCarDrivingStatus()
                        Timber.tag("drivingstatus").d("탑승 ${_drivingStatus.value}")
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

    fun requestRentCompleteToday(rentId: Long) {
        viewModelScope.launch {
            val request = RequestCarStatus(
                rentId.toInt(),
                _rentTravelInfo.value?.travelId ?: -1,
                _rentTravelInfo.value?.travelDatesId ?: -1,
                _rentTravelInfo.value?.datePlacesId ?: -1
            )
            rentRepository.completeTodayRent(
                request
            ).collect {
                it.onSuccess {
                    Timber.tag("today").d("오늘 렌트 끝내기 완료 $request")
                    _rentCompleteToday.emit(true)
                }
                it.onFailure {
                    Timber.tag("today").d("오늘 렌트 끝내기 실패 $request")
                    _rentCompleteToday.emit(false)
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
                        stopGPSPublish()
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
                        getCarDrivingStatus()
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

    private fun getReverseGeocode(latitude: Double, longitude: Double) {
        viewModelScope.launch {
            naverRepository.getReverseGeocode(latitude, longitude).collect { result ->
                result.onSuccess { data ->
                    _reverseGeocode.emit(Result.success(formatAddress(data)))
                }.onFailure {
                    _reverseGeocode.emit(Result.failure(Exception("fail")))
                }
            }
        }
    }

    fun clearReverseGeocode() {
        _reverseGeocode.value = null
    }

    private fun formatAddress(response: ResponseReverseGeocode): String {
        val result = response.results.firstOrNull()
        return if (result != null) {
            val region = result.region
            val land = result.land
            "${region.area1.name} ${region.area2.name} ${region.area3.name} ${land?.addition0?.value}"
        } else {
            "주소를 찾을 수 없습니다"
        }
    }

    // 첫 호출 리팩토링 -> 일정변경하기
    fun modifyFirstEvent() {
        viewModelScope.launch {
            val latlng = _destination.value ?: return@launch
            val rent = _reservationRent.value ?: return@launch
            rentRepository.getRentDetail(rent.rentId).collect { detail ->
                detail.onSuccess { result ->
                    planRepository.getPlanDetail(result.travelId.toInt())
                        .collect { planResult ->
                            planResult.onSuccess { plan ->
                                val planList = plan.datesDetail.toMutableList()
                                // 첫 일정이 다음 일정으로 밀려야 됨
                                val firstDayPlan = plan.datesDetail.first() // 첫 호출이니까
                                val addressName = _reverseGeocode.value?.getOrNull()
                                val buildingName = addressName?.split(" ")?.last()
                                Timber.tag("수정").d("$addressName\n $buildingName")
                                val placeList =
                                    plan.datesDetail.first().placesDetail.toMutableList()
                                val modifiedFirstPlanItem =
                                    firstDayPlan.placesDetail.first().copy(
                                        datePlacesName = buildingName ?: "임시 장소",
                                        datePlacesAddress = addressName ?: "임시 주소",
                                        datePlacesLat = latlng.latitude,
                                        datePlacesLon = latlng.longitude
                                    )
                                val newPlanItemList = mutableListOf<PlanItem>()
                                newPlanItemList.add(modifiedFirstPlanItem)
                                newPlanItemList.add(placeList[0])
                                val modifiedDayPlan = firstDayPlan.copy(
                                    placesDetail = newPlanItemList
                                )
                                planList[0] = modifiedDayPlan
                                planRepository.putPlan(
                                    Plan(
                                        travelId = plan.travelId,
                                        travelName = plan.travelName,
                                        datesDetail = planList.toList(),
                                        travelStartDate = plan.travelStartDate,
                                        travelEndDate = plan.travelEndDate,
                                    )
                                ).collect { planModify ->
                                    planModify.onSuccess { result ->
                                        Timber.tag("Car").d("수정 성공 $result")
                                        rentCarRepository.setCarWithTravelInfo(
                                            RequestCarStatus(
                                                rentId = rent.rentId.toInt(),
                                                travelId = result.travelId,
                                                travelDatesId = result.travelDatesId,
                                                datePlacesId = result.datePlacesId
                                            )
                                        )
                                        callFirstAssignedCar()
                                    }.onFailure { e ->
                                        Timber.tag("Car").d("수정 실패 $e")
                                    }
                                }
                                Timber.tag("Car").d("$plan")
                            }.onFailure { e ->
                                Timber.tag("Car").d("$e")
                            }
                        }
                }.onFailure { e ->
                    Timber.tag("Car").d("$e")
                }
            }
        }
    }

    companion object {
        private const val DEFAULT_INTERVAL = 1000L
        private const val GPS_SUB = "gps/data/v1/subscribe"
        private const val PATH_SUB = "path/data/v1/subscribe"
//        private const val ORIENTATION_SUB = "orientation/data/v1/subscribe"
    }
}