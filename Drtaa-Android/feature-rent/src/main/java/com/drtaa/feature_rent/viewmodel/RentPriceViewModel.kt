package com.drtaa.feature_rent.viewmodel

import androidx.lifecycle.ViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import javax.inject.Inject

class RentPriceViewModel @Inject constructor() : ViewModel() {
    private val _rentIsHour = MutableStateFlow<Boolean?>(null)
    val rentIsHour: StateFlow<Boolean?> = _rentIsHour

    fun setRentIsHour(isHour: Boolean) {
        _rentIsHour.value = isHour
    }
}