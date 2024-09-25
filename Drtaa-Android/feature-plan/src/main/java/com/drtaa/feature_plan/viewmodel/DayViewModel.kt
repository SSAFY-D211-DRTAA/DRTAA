package com.drtaa.feature_plan.viewmodel

import androidx.lifecycle.ViewModel
import com.drtaa.core_model.plan.Plan
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import javax.inject.Inject

@HiltViewModel
class DayViewModel @Inject constructor() : ViewModel() {

    private var day = 0

    private val _dayPlan = MutableStateFlow<Plan.DayPlan?>(null)
    val dayPlan: StateFlow<Plan.DayPlan?> = _dayPlan

    fun setDayAndPlan(day: Int, plan: Plan.DayPlan?) {
        this.day = day
        _dayPlan.value = plan
    }
}