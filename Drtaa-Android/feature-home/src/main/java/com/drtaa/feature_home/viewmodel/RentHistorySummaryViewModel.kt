package com.drtaa.feature_home.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_model.rent.RentDetail
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class RentHistorySummaryViewModel @Inject constructor(
    private val rentRepository: RentRepository
) : ViewModel() {
    private val _rentId = MutableStateFlow<Long?>(null)
    val rentId: StateFlow<Long?> = _rentId

    private val _rentSummary = MutableStateFlow<RentDetail?>(null)
    val rentSummary: StateFlow<RentDetail?> = _rentSummary

    init {
        initObserve()
    }

    fun setRentId(rentId: Long) {
        _rentId.value = rentId
    }

    private fun getRentSummary(rentId: Long) {
        viewModelScope.launch {
            rentRepository.getRentDetail(rentId).collect { result ->
                result.onSuccess { data ->
                    _rentSummary.value = data
                }.onFailure {
                    _rentSummary.value = null
                }
            }
        }
    }

    private fun initObserve() {
        viewModelScope.launch {
            rentId.collect {
                if (it != null) {
                    getRentSummary(it)
                }
            }
        }
    }
}