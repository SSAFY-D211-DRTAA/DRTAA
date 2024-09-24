package com.drtaa.feature_rent.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_model.map.Search
import com.drtaa.core_model.network.RequestDuplicatedSchedule
import com.drtaa.core_model.rent.RentInfo
import com.drtaa.core_model.rent.RentSchedule
import com.drtaa.core_model.util.toLocalDateTime
import com.naver.maps.geometry.LatLng
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.launch
import java.time.Duration
import javax.inject.Inject

@HiltViewModel
class RentViewModel @Inject constructor(
    private val rentRepository: RentRepository
) : ViewModel() {
    private val _rentStartLocation = MutableStateFlow<Search?>(null)
    val rentStartLocation: StateFlow<Search?> = _rentStartLocation

    private val _rentStartSchedule = MutableStateFlow<RentSchedule?>(null)
    val rentStartSchedule: StateFlow<RentSchedule?> = _rentStartSchedule

    private val _rentEndSchedule = MutableStateFlow<RentSchedule?>(null)
    val rentEndSchedule: StateFlow<RentSchedule?> = _rentEndSchedule

    private val _rentPeople = MutableStateFlow(1)
    val rentPeople: StateFlow<Int> = _rentPeople

    private val _isRentValid = MutableStateFlow(false)
    val isRentValid: StateFlow<Boolean> = _isRentValid

    private val _rentInfo = MutableStateFlow<RentInfo?>(null)
    val rentInfo: StateFlow<RentInfo?> = _rentInfo

    private val _isDuplicatedSchedule = MutableSharedFlow<Boolean?>()
    val isDuplicatedSchedule: SharedFlow<Boolean?> = _isDuplicatedSchedule

    val DEFAULT_LATLNG = LatLng(37.57578754990568, 126.90027478459672)

    init {
        setRentValid()
    }

    fun setRentStartLocation(search: Search) {
        viewModelScope.launch {
            val jungryujang = Search(
                title = search.title,
                category = search.category,
                roadAddress = search.roadAddress,
                lng = DEFAULT_LATLNG.longitude,
                lat = DEFAULT_LATLNG.latitude
            )
//            _rentStartLocation.value = search
            _rentStartLocation.value = jungryujang
        }
    }

    fun setRentStartSchedule(startSchedule: RentSchedule) {
        viewModelScope.launch {
            _rentStartSchedule.value = startSchedule
        }
    }

    fun setRentEndSchedule(endSchedule: RentSchedule) {
        viewModelScope.launch {
            _rentEndSchedule.value = endSchedule
        }
    }

    fun increaseRentPeople(): Boolean {
        var result = false
        viewModelScope.launch {
            if (_rentPeople.value < MAX_PEOPLE) {
                _rentPeople.value++
                result = true
            }
        }

        return result
    }

    fun decreaseRentPeople(): Boolean {
        var result = false
        viewModelScope.launch {
            if (_rentPeople.value > MIN_PEOPLE) {
                _rentPeople.value--
                result = true
            }
        }
        return result
    }

    fun checkDuplicatedSchedule() {
        viewModelScope.launch {
            rentRepository.checkDuplicatedRent(
                RequestDuplicatedSchedule(
                    rentStartTime = rentStartSchedule.value!!.toRequestUnassignedCar(),
                    rentEndTime = rentEndSchedule.value!!.toRequestUnassignedCar()
                )
            ).collect { result ->
                result.onSuccess { data ->
                    _isDuplicatedSchedule.emit(data)
                }.onFailure {
                    _isDuplicatedSchedule.emit(null)
                }
            }
        }
    }

    private fun setRentValid() {
        viewModelScope.launch {
            combine(
                rentStartLocation,
                rentStartSchedule,
                rentEndSchedule,
                rentPeople
            ) { rentStartLocation, rentStartSchedule, rentEndSchedule, rentPeople ->
                rentStartLocation != null && rentStartSchedule != null && rentEndSchedule != null && rentPeople > 0
            }.collect { isValid ->
                _isRentValid.value = isValid
            }
        }
    }

    fun getRentInfo() {
        viewModelScope.launch {
            val hours = calculateHours()
            val price = (hours * PRICE_PER_HOUR).toInt()
            val discount = -1 * ((hours.toInt() / ONE_DAY) * DISCOUNT_PER_DAY)
            val finalPrice = price + discount

            _rentInfo.emit(
                RentInfo(
                    carInfo = null,
                    hours = hours,
                    price = price,
                    discount = discount,
                    finalPrice = finalPrice,
                    people = _rentPeople.value,
                    startLocation = _rentStartLocation.value!!,
                    startSchedule = _rentStartSchedule.value!!,
                    endSchedule = _rentEndSchedule.value!!,
                )
            )
        }
    }

    private fun calculateHours(): Double {
        val startDateTime = _rentStartSchedule.value!!.toLocalDateTime()
        val endDateTime = _rentEndSchedule.value!!.toLocalDateTime()

        val duration = Duration.between(startDateTime, endDateTime)
        val hours = duration.toHours()
        val minutes = duration.toMinutes() % ONE_HOUR_TO_MINUTE / ONE_HOUR_TO_MINUTE.toDouble()

        return hours.toInt() + minutes
    }

    companion object {
        private const val MIN_PEOPLE = 1
        private const val MAX_PEOPLE = 8

        private const val PRICE_PER_HOUR = 20000
        private const val DISCOUNT_PER_DAY = 240000

        private const val ONE_DAY = 24
        private const val ONE_HOUR_TO_MINUTE = 60
    }
}