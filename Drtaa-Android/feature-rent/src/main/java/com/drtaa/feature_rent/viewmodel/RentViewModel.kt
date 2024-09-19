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

    private val _isRentValid = MutableStateFlow(false)
    val isRentValid: StateFlow<Boolean> = _isRentValid

    private val _rentInfo = MutableStateFlow<Result<RentInfo>?>(null)
    val rentInfo: StateFlow<Result<RentInfo>?> = _rentInfo

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
            // api 작업
            _rentInfo.emit(
                Result.success(
                    RentInfo(
                        isAvailable = true,
                        carName = "Genesis G80",
                        fareType = "1시간권",
                        fareCount = 8,
                        price = "120,000",
                        discount = "5,000",
                        totalPrice = "115,000",
                        startDate = "09.17(화)",
                        endDate = "09.21(금)",
                        startTime = "10:00",
                        endTime = "18:00",
                        people = 3,
                        startLocation = "서울특별시 강남구 역삼동",
                        carImg = ""
                    )
                )
            )

//            _rentInfo.emit(
//                Result.failure(
//                    Exception("api 실패")
//                )
//            )
        }
    }

    companion object {
        private const val MIN_PEOPLE = 1
        private const val MAX_PEOPLE = 8
    }
}