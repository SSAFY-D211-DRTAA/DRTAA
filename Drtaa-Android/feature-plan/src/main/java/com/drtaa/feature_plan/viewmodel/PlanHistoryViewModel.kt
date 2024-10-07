package com.drtaa.feature_plan.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.PlanRepository
import com.drtaa.core_model.plan.PlanSimple
import com.drtaa.core_model.util.Status
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class PlanHistoryViewModel @Inject constructor(
    private val planRepository: PlanRepository
) : ViewModel() {
    private val _planList = MutableStateFlow<List<PlanSimple>?>(null)
    val planList: StateFlow<List<PlanSimple>?> = _planList

    private val _planReservedList = MutableStateFlow<List<PlanSimple>?>(null)
    val planReservedList: StateFlow<List<PlanSimple>?> = _planReservedList

    private val _planCompletedList = MutableStateFlow<List<PlanSimple>?>(null)
    val planCompletedList: StateFlow<List<PlanSimple>?> = _planCompletedList

    private val _planInProgress = MutableStateFlow<PlanSimple?>(null)
    val planInProgress: StateFlow<PlanSimple?> = _planInProgress

    init {
        initObserve()
        getPlanList()
    }

    fun getPlanList() {
        viewModelScope.launch {
            planRepository.getPlanList().collect { result ->
                result.onSuccess { data ->
                    Timber.tag("search").d("success $data")
                    _planList.value = data
                }.onFailure {
                    Timber.tag("search").d("fail")
                    _planList.value = null
                }
            }
        }
    }

    private fun initObserve() {
        viewModelScope.launch {
            _planList.collect { planList ->
                if (planList == null) {
                    _planReservedList.value = null
                    _planCompletedList.value = null
                    return@collect
                }

                _planInProgress.value = planList.find { it.rentStatus == Status.IN_PROGRESS.status }
                _planReservedList.value =
                    planList.filter { it.rentStatus == Status.RESERVED.status }
                        .sortedByDescending { it.travelEndDate }
                _planCompletedList.value =
                    planList.filter { it.rentStatus == Status.COMPLETED.status }
                        .sortedByDescending { it.travelEndDate }
            }
        }
    }
}