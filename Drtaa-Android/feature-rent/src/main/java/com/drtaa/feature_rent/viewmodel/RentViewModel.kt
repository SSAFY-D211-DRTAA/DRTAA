package com.drtaa.feature_rent.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_model.rent.RentInfo
import com.drtaa.core_model.rent.RentSchedule
import com.drtaa.core_model.map.Search
import com.drtaa.core_model.util.toLocalDateTime
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.launch
import timber.log.Timber
import java.time.Duration
import javax.inject.Inject

@HiltViewModel
class RentViewModel @Inject constructor() : ViewModel() {
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

    init {
        setRentValid()
    }

    fun setRentStartLocation(search: Search) {
        viewModelScope.launch {
            _rentStartLocation.value = search
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
            val discount = -1 * ((hours.toInt() / 24) * DISCOUNT_PER_DAY)
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
        val minutes = duration.toMinutes() % 60 / 60.0

        return hours.toInt() + minutes
    }

    companion object {
        private const val MIN_PEOPLE = 1
        private const val MAX_PEOPLE = 8

        private const val PRICE_PER_HOUR = 20000
        private const val PRICE_PER_DAY = 200000
        private const val DISCOUNT_PER_DAY = 240000
    }
}