package com.drtaa.feature_rent.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_model.data.Time
import com.prolificinteractive.materialcalendarview.CalendarDay
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class CalendarBottomSheetViewModel @Inject constructor() : ViewModel() {
    private val _rentStartDate = MutableStateFlow<CalendarDay?>(null)
    val rentStartDate: StateFlow<CalendarDay?> = _rentStartDate

    private val _rentEndDate = MutableStateFlow<CalendarDay?>(null)
    val rentEndDate: StateFlow<CalendarDay?> = _rentEndDate

    private val _rentStartTime = MutableStateFlow<Time?>(null)
    val rentStartTime: StateFlow<Time?> = _rentStartTime

    private val _rentEndTime = MutableStateFlow<Time?>(null)
    val rentEndTime: StateFlow<Time?> = _rentEndTime

    private val _isScheduleValid = MutableStateFlow(false)
    val isScheduleValid: StateFlow<Boolean> = _isScheduleValid

    init {
        setScheduleValid()
    }

    fun setRentStartDate(startDate: CalendarDay?) {
        _rentStartDate.value = startDate
    }

    fun setRentEndDate(endDate: CalendarDay?) {
        _rentEndDate.value = endDate
    }

    fun setRentStartTime(hour: Int, minute: Int) {
        _rentStartTime.value = Time(hour, minute)
    }

    fun setRentEndTime(hour: Int, minute: Int) {
        _rentEndTime.value = Time(hour, minute)
    }

    private fun setScheduleValid() {
        viewModelScope.launch {
            combine(
                rentStartDate,
                rentEndDate,
                rentStartTime,
                rentEndTime
            ) { startDate, endDate, rentStartTime, rentEndTime ->
                startDate != null && endDate != null && rentStartTime != null && rentEndTime != null
            }.collect { isValid ->
                _isScheduleValid.value = isValid
            }
        }
    }
}