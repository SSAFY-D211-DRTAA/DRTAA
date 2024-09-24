package com.drtaa.feature_plan.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_model.plan.Plan
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.stateIn
import javax.inject.Inject

@HiltViewModel
class DayViewModel @Inject constructor(
    private val planViewModel: PlanViewModel
) : ViewModel() {

    private var day = 0

    // dayPlanFlow를 PlanViewModel에서 데이터를 받아 처리
    private val _dayPlan = planViewModel.plan
        .map { plan ->
            plan?.datesDetail?.find { it.travelDatesId == day }
        }
        .stateIn(
            scope = viewModelScope,
            started = SharingStarted.WhileSubscribed(5000),
            initialValue = null
        )
    val dayPlan: StateFlow<Plan.DayPlan?> = _dayPlan

    fun setDay(day: Int) {
        this.day = day
    }
}