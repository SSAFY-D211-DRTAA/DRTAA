package com.drtaa.feature_rent.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_model.network.ResponseRentCar
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class RentSummaryViewModel @Inject constructor(
    private val rentRepository: RentRepository
) : ViewModel() {
    private val _assignedCar = MutableSharedFlow<ResponseRentCar?>()
    val assignedCar: SharedFlow<ResponseRentCar?> = _assignedCar

    fun getUnAssignedCar() {
        viewModelScope.launch {
            rentRepository.getUnassignedCar().collect { result ->
                result.onSuccess { data ->
                    Timber.d("성공")
                    _assignedCar.emit(data)
                }.onFailure {
                    Timber.d("실패")
                    _assignedCar.emit(null)
                }
            }
        }
    }
}