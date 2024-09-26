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

//    private var seoulTrip: Plan = Plan(
//        travelName = "서울 여행",
//        travelStartDate = "2024-09-23",
//        travelEndDate = "2024-09-24",
//        datesDetail = listOf(
//            Plan.DayPlan(
//                travelDatesId = 1,
//                travelDatesDate = "2024-09-23",
//                placesDetail = listOf(
//                    Plan.DayPlan.PlanItem(
//                        datePlacesId = 1,
//                        datePlacesName = "경복궁",
//                        datePlacesCategory = "문화재",
//                        datePlacesAddress = "서울특별시 종로구 사직로 161",
//                        datePlacesLat = 37.5772,
//                        datePlacesLon = 126.9768,
//                        datePlacesIsVisited = false
//                    ),
//                    Plan.DayPlan.PlanItem(
//                        datePlacesId = 2,
//                        datePlacesName = "북촌 한옥마을",
//                        datePlacesCategory = "관광지",
//                        datePlacesAddress = "서울특별시 종로구 계동길 37",
//                        datePlacesLat = 37.5822,
//                        datePlacesLon = 126.983,
//                        datePlacesIsVisited = false
//                    ),
//                    Plan.DayPlan.PlanItem(
//                        datePlacesId = 3,
//                        datePlacesName = "남산타워",
//                        datePlacesCategory = "관광지",
//                        datePlacesAddress = "서울특별시 용산구 남산공원길 105",
//                        datePlacesLat = 37.5512,
//                        datePlacesLon = 126.9881,
//                        datePlacesIsVisited = false
//                    ),
//                    Plan.DayPlan.PlanItem(
//                        datePlacesId = 4,
//                        datePlacesName = "명동",
//                        datePlacesCategory = "쇼핑몰",
//                        datePlacesAddress = "서울특별시 중구 명동8나길 27",
//                        datePlacesLat = 37.5633,
//                        datePlacesLon = 126.982,
//                        datePlacesIsVisited = false
//                    ),
//                    Plan.DayPlan.PlanItem(
//                        datePlacesId = 5,
//                        datePlacesName = "홍대",
//                        datePlacesCategory = "관광지",
//                        datePlacesAddress = "서울특별시 마포구 양화로 177",
//                        datePlacesLat = 37.5561,
//                        datePlacesLon = 126.9257,
//                        datePlacesIsVisited = false
//                    )
//                )
//            ),
//            Plan.DayPlan(
//                travelDatesId = 2,
//                travelDatesDate = "2024-09-24",
//                placesDetail = listOf(
//                    Plan.DayPlan.PlanItem(
//                        datePlacesId = 6,
//                        datePlacesName = "디지털미디어시티역",
//                        datePlacesCategory = "지하철역",
//                        datePlacesAddress = "서울특별시 마포구 월드컵북로 366",
//                        datePlacesLat = 0.0,
//                        datePlacesLon = 0.0,
//                        datePlacesIsVisited = false
//                    )
//                )
//            )
//        )
//    )

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
        val currentPlan = _plan.value
        if (currentPlan == null) {
//            _isEditSuccess.value = false
            return
        }

        val updatedDatesDetail = currentPlan.datesDetail.map { dayPlan ->
            if (dayPlan.travelDatesId == dayIdx + 1) {
                val newPlanList = dayPlan.placesDetail.toMutableList().apply {
                    addAll(addPlanList)
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
