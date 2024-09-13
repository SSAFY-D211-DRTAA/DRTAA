package com.drtaa.feature_rent.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_model.data.Search
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class RentViewModel @Inject constructor() : ViewModel() {
    private val _rentStartLocation = MutableStateFlow<Search?>(null)
    val rentStartLocation: StateFlow<Search?> = _rentStartLocation

    private val _rentStartDate = MutableStateFlow<String?>(null)
    val rentStartDate: StateFlow<String?> = _rentStartDate

    private val _rentEndDate = MutableStateFlow<String?>(null)
    val rentEndDate: StateFlow<String?> = _rentEndDate

    fun setRentStartLocation(search: Search) {
        viewModelScope.launch {
            _rentStartLocation.value = search
        }
    }

    fun setRentDate(startDate: String, endDate: String) {
        viewModelScope.launch {
            _rentStartDate.value = startDate
            _rentEndDate.value = endDate
        }
    }
}