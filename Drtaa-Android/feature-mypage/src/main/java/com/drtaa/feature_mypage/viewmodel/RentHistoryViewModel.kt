package com.drtaa.feature_mypage.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_model.rent.RentSimple
import com.drtaa.core_model.util.Status
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class RentHistoryViewModel @Inject constructor(
    private val rentRepository: RentRepository
) : ViewModel() {
    private val _rentList = MutableStateFlow<List<RentSimple>?>(null)
    val rentList: StateFlow<List<RentSimple>?> = _rentList

    private val _rentReservedList = MutableStateFlow<List<RentSimple>?>(null)
    val rentReservedList: StateFlow<List<RentSimple>?> = _rentReservedList

    private val _rentCompletedList = MutableStateFlow<List<RentSimple>?>(null)
    val rentCompletedList: StateFlow<List<RentSimple>?> = _rentCompletedList

    private val _rentInProgress = MutableStateFlow<RentSimple?>(null)
    val rentInProgress: StateFlow<RentSimple?> = _rentInProgress

    init {
        initObserve()
        getRentList()
    }

    fun getRentList() {
        viewModelScope.launch {
            rentRepository.getRentHistory().collect { result ->
                result.onSuccess { data ->
                    _rentList.value = data
                    Timber.d("렌트 기록 조회 성공")
                }.onFailure {
                    Timber.d("렌트 기록 조회 실패")
                }
            }
        }
    }

    private fun initObserve() {
        viewModelScope.launch {
            _rentList.collect { rentList ->
                if (rentList == null) {
                    _rentReservedList.value = null
                    _rentCompletedList.value = null
                    return@collect
                }

                _rentInProgress.value = rentList.find { it.rentStatus == Status.IN_PROGRESS.status }
                _rentReservedList.value =
                    rentList.filter { it.rentStatus == Status.RESERVED.status }
                        .sortedByDescending { it.rentEndTime }
                _rentCompletedList.value =
                    rentList.filter { it.rentStatus == Status.COMPLETED.status }
                        .sortedByDescending { it.rentEndTime }
            }
        }
    }
}