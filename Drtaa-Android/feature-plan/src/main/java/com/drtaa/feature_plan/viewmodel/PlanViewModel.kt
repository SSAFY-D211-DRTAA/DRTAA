package com.drtaa.feature_plan.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.PlanRepository
import com.drtaa.core_model.plan.Plan
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import java.util.Collections.addAll
import javax.inject.Inject

@HiltViewModel
class PlanViewModel @Inject constructor(
    private val planRepository: PlanRepository
) : ViewModel() {
    private var travelId: Int = 0

    private val _plan = MutableStateFlow<Plan?>(null)
    val plan: StateFlow<Plan?> = _plan

    private val _dayPlanList = MutableStateFlow<List<Plan.DayPlan>?>(null)
    val dayPlanList: StateFlow<List<Plan.DayPlan>?> = _dayPlanList

    private val _isEditMode = MutableStateFlow(false)
    val isEditMode: StateFlow<Boolean> = _isEditMode

    private val _isEditSuccess = MutableStateFlow<Boolean?>(null)
    val isEditSuccess: StateFlow<Boolean?> = _isEditSuccess

    var isViewPagerLoaded = false

    init {
        observePlan()
    }

    fun setTravelId(travelId: Int) {
        this.travelId = travelId
    }

    fun getPlan() {
        viewModelScope.launch {
            planRepository.getPlanDetail(travelId).collect { result ->
                result.onSuccess { data ->
                    _plan.value = data
                    Timber.d("getPlan 성공 $data")
                }.onFailure {
                    _plan.value = null
                }
            }
        }
    }

    fun setIsEditSuccess(isEditSuccess: Boolean?) {
        _isEditSuccess.value = isEditSuccess
    }

    fun setEditMode(isEditMode: Boolean) {
        _isEditMode.value = isEditMode

        if (!isEditMode) {
            val viewModePlan = _plan.value?.datesDetail?.map { dayPlan ->
                dayPlan.copy(
                    placesDetail = dayPlan.placesDetail.map { planItem ->
                        planItem.copy(isSelected = false)
                    }
                )
            }

            if (viewModePlan != null) {
                _plan.value = _plan.value?.copy(datesDetail = viewModePlan)
            }
        }
    }

    fun addPlan(dayIdx: Int, addPlanList: List<Plan.DayPlan.PlanItem>) {
        Timber.d("addPlan $dayIdx $addPlanList")
        val currentPlan = _plan.value
        if (currentPlan == null) {
//            _isEditSuccess.value = false
            return
        }

        val updatedDatesDetail = currentPlan.datesDetail.mapIndexed { index, dayPlan ->
            if (index == dayIdx) {
                val newPlanList = dayPlan.placesDetail.toMutableList().apply {
                    addAll(addPlanList)
                }

                dayPlan.copy(placesDetail = newPlanList)
            } else {
                dayPlan
            }
        }

        Timber.d("updatedDatesDetail $updatedDatesDetail")

        if (updatedDatesDetail == currentPlan.datesDetail) {
            _isEditSuccess.value = false
            return
        }

        val tempNewPlan = currentPlan.copy(datesDetail = updatedDatesDetail)
        putNewPlan(tempNewPlan)
    }

    fun updateDate(
        dayIdxFrom: Int,
        dayIdxTo: Int,
        movePlanList: List<Plan.DayPlan.PlanItem>
    ) {
        val currentPlan = _plan.value
        if (currentPlan == null) {
//            _isEditSuccess.value = false
            return
        }

        val updatedDatesDetail = currentPlan.datesDetail.map { dayPlan ->
            when (dayPlan.travelDatesId) {
                dayIdxFrom + 1 -> {
                    val newPlanList = dayPlan.placesDetail.toMutableList().apply {
                        removeAll(movePlanList)
                    }

                    dayPlan.copy(placesDetail = newPlanList)
                }

                dayIdxTo + 1 -> {
                    val newPlanList = dayPlan.placesDetail.toMutableList().apply {
                        addAll(movePlanList)
                    }

                    dayPlan.copy(placesDetail = newPlanList)
                }

                else -> {
                    dayPlan
                }
            }
        }

        if (updatedDatesDetail == currentPlan.datesDetail) {
            _isEditSuccess.value = false
            return
        }

        val tempNewPlan = currentPlan.copy(datesDetail = updatedDatesDetail)
        putNewPlan(tempNewPlan)
    }

    fun updatePlan(dayIdx: Int, newPlanList: List<Plan.DayPlan.PlanItem>) {
        val currentPlan = _plan.value
        if (currentPlan == null) {
//            _isEditSuccess.value = false
            return
        }

        val updatedDatesDetail = currentPlan.datesDetail.map { dayPlan ->
            if (dayPlan.travelDatesId == dayIdx + 1) {
                dayPlan.copy(placesDetail = newPlanList)
            } else {
                dayPlan
            }
        }

        if (updatedDatesDetail == currentPlan.datesDetail) {
            _isEditSuccess.value = false
            return
        }

        val tempNewPlan = currentPlan.copy(datesDetail = updatedDatesDetail)
        putNewPlan(tempNewPlan)
    }

    fun deletePlan(dayIdx: Int, deletedPlanList: List<Plan.DayPlan.PlanItem>) {
        val currentPlan = _plan.value
        if (currentPlan == null) {
//            _isEditSuccess.value = false
            return
        }

        val updatedDatesDetail = currentPlan.datesDetail.map { dayPlan ->
            if (dayPlan.travelDatesId == dayIdx + 1) {
                val newPlanList = dayPlan.placesDetail.toMutableList().apply {
                    removeAll(deletedPlanList)
                }

                dayPlan.copy(placesDetail = newPlanList)
            } else {
                dayPlan
            }
        }

        if (updatedDatesDetail == currentPlan.datesDetail) {
            _isEditSuccess.value = false
            return
        }

        val tempNewPlan = currentPlan.copy(datesDetail = updatedDatesDetail)
        putNewPlan(tempNewPlan)
    }

    private fun putNewPlan(plan: Plan) {
        Timber.tag("NEW_PLAN").d("$plan")
        viewModelScope.launch {
            planRepository.putPlan(plan).collect { response ->
                response.onSuccess {
                    _plan.value = plan
                    _isEditSuccess.value = true
                    Timber.tag("NEW_PLAN").d("${_plan.value}")
                }.onFailure {
                    _isEditSuccess.value = false
                }
            }
        }
    }

    private fun observePlan() {
        viewModelScope.launch {
            plan.collect { plan ->
                _dayPlanList.value = plan?.datesDetail
            }
        }
    }
}
