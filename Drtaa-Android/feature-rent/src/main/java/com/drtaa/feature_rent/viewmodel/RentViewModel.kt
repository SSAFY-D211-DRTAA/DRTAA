package com.drtaa.feature_rent.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_model.rent.RentInfo
import com.drtaa.core_model.rent.RentSchedule
import com.drtaa.core_model.map.Search
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.launch
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

    private val _rentIsHour = MutableStateFlow<Boolean?>(null)
    val rentIsHour: StateFlow<Boolean?> = _rentIsHour

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

    fun setRentIsHour(isHour: Boolean) {
        _rentIsHour.value = isHour
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
            _rentInfo.emit(
                RentInfo(
                    carInfo = null,
                    isHour = _rentIsHour.value!!,
                    fareCount = 8,
                    price = 120000,
                    discount = 5000,
                    totalPrice = 115000,
                    people = _rentPeople.value,
                    startLocation = _rentStartLocation.value!!,
                    startSchedule = _rentStartSchedule.value!!,
                    endSchedule = _rentEndSchedule.value!!,
                )
            )
        }
    }

    companion object {
        private const val MIN_PEOPLE = 1
        private const val MAX_PEOPLE = 8
    }
}