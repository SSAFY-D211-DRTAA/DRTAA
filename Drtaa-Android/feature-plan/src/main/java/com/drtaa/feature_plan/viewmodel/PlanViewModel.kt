package com.drtaa.feature_plan.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_model.plan.Plan
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class PlanViewModel @Inject constructor() : ViewModel() {
    private val _plan = MutableStateFlow<Plan?>(null)
    val plan: StateFlow<Plan?> = _plan

    private val _dayPlanList = MutableStateFlow<List<Plan.DayPlan>?>(null)
    val dayPlanList: StateFlow<List<Plan.DayPlan>?> = _dayPlanList

    private val _isEditMode = MutableStateFlow(false)
    val isEditMode: StateFlow<Boolean> = _isEditMode

    var isViewPagerLoaded = false

    private var seoulTrip: Plan = Plan(
        travelName = "서울 여행",
        travelStartDate = "2024-09-23",
        travelEndDate = "2024-09-24",
        datesDetail = listOf(
            Plan.DayPlan(
                travelDatesId = 1,
                travelDatesDate = "2024-09-23",
                placesDetail = listOf(
                    Plan.DayPlan.PlanItem(
                        datePlacesId = 1,
                        datePlacesName = "경복궁",
                        datePlacesCategory = "문화재",
                        datePlacesAddress = "서울특별시 종로구 사직로 161",
                        datePlacesLat = 37.5772,
                        datePlacesLon = 126.9768,
                        datePlacesIsVisited = false
                    ),
                    Plan.DayPlan.PlanItem(
                        datePlacesId = 2,
                        datePlacesName = "북촌 한옥마을",
                        datePlacesCategory = "관광지",
                        datePlacesAddress = "서울특별시 종로구 계동길 37",
                        datePlacesLat = 37.5822,
                        datePlacesLon = 126.983,
                        datePlacesIsVisited = false
                    ),
                    Plan.DayPlan.PlanItem(
                        datePlacesId = 3,
                        datePlacesName = "남산타워",
                        datePlacesCategory = "관광지",
                        datePlacesAddress = "서울특별시 용산구 남산공원길 105",
                        datePlacesLat = 37.5512,
                        datePlacesLon = 126.9881,
                        datePlacesIsVisited = false
                    ),
                    Plan.DayPlan.PlanItem(
                        datePlacesId = 4,
                        datePlacesName = "명동",
                        datePlacesCategory = "쇼핑몰",
                        datePlacesAddress = "서울특별시 중구 명동8나길 27",
                        datePlacesLat = 37.5633,
                        datePlacesLon = 126.982,
                        datePlacesIsVisited = false
                    ),
                    Plan.DayPlan.PlanItem(
                        datePlacesId = 5,
                        datePlacesName = "홍대",
                        datePlacesCategory = "관광지",
                        datePlacesAddress = "서울특별시 마포구 양화로 177",
                        datePlacesLat = 37.5561,
                        datePlacesLon = 126.9257,
                        datePlacesIsVisited = false
                    )
                )
            ),
            Plan.DayPlan(
                travelDatesId = 2,
                travelDatesDate = "2024-09-24",
                placesDetail = listOf(
                    Plan.DayPlan.PlanItem(
                        datePlacesId = 6,
                        datePlacesName = "디지털미디어시티역",
                        datePlacesCategory = "지하철역",
                        datePlacesAddress = "서울특별시 마포구 월드컵북로 366",
                        datePlacesLat = 0.0,
                        datePlacesLon = 0.0,
                        datePlacesIsVisited = false
                    )
                )
            )
        )
    )

    init {
        observePlan()
        getPlan()
    }

    private fun getPlan() {
        _plan.value = seoulTrip
        Timber.d("getPlan ${_plan.value}")
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

            if (viewModePlan != null){
                _plan.value = _plan.value?.copy(datesDetail = viewModePlan)
            }
        }
    }

    fun addPlan(dayIdx: Int, addPlanList: List<Plan.DayPlan.PlanItem>): Boolean {
        var isSuccess = false

        // 현재 _plan 값을 가져옴
        val currentPlan = _plan.value

        currentPlan?.let { plan ->
            // datesDetail 리스트에서 해당 day에 맞는 DayPlan을 찾아 수정
            val updatedDatesDetail = plan.datesDetail.map { dayPlan ->
                if (dayPlan.travelDatesId == dayIdx + 1) {
                    val newPlanList = dayPlan.placesDetail.toMutableList()
                    isSuccess = newPlanList.addAll(addPlanList)

                    dayPlan.copy(placesDetail = newPlanList)
                } else {
                    dayPlan
                }
            }

            if (!isSuccess) {
                return false
            }

            // 서버에 저장시키는 코드 추가
            // 수정된 datesDetail을 갖는 새로운 Plan 객체를 _plan에 방출
            _plan.value = plan.copy(datesDetail = updatedDatesDetail)
            Timber.d("delete ${_plan.value}")
        }
        return isSuccess
    }

    fun updatePlan(dayIdx: Int, newPlanList: List<Plan.DayPlan.PlanItem>): Boolean {
        var isSuccess = false

        // 현재 _plan 값을 가져옴
        val currentPlan = _plan.value

        currentPlan?.let { plan ->
            // datesDetail 리스트에서 해당 day에 맞는 DayPlan을 찾아 수정
            val updatedDatesDetail = plan.datesDetail.map { dayPlan ->
                if (dayPlan.travelDatesId == dayIdx + 1) {
                    dayPlan.copy(placesDetail = newPlanList)
                } else {
                    dayPlan
                }
            }

            // 서버에 저장시키는 코드 추가
            // 수정된 datesDetail을 갖는 새로운 Plan 객체를 _plan에 방출
            _plan.value = plan.copy(datesDetail = updatedDatesDetail)
            Timber.tag("update plan").d("updated plan ${_plan.value}")
        }
        return isSuccess
    }

    fun deletePlan(dayIdx: Int, deletedPlanList: List<Plan.DayPlan.PlanItem>): Boolean {
        var isSuccess = false

        // 현재 _plan 값을 가져옴
        val currentPlan = _plan.value

        currentPlan?.let { plan ->
            // datesDetail 리스트에서 해당 day에 맞는 DayPlan을 찾아 수정
            val updatedDatesDetail = plan.datesDetail.map { dayPlan ->
                if (dayPlan.travelDatesId == dayIdx + 1) {
                    val newPlanList = dayPlan.placesDetail.toMutableList()
                    isSuccess = newPlanList.removeAll(deletedPlanList)

                    dayPlan.copy(placesDetail = newPlanList)
                } else {
                    dayPlan
                }
            }

            if (!isSuccess) {
                return false
            }

            // 서버에 저장시키는 코드 추가
            // 수정된 datesDetail을 갖는 새로운 Plan 객체를 _plan에 방출
            _plan.value = plan.copy(datesDetail = updatedDatesDetail)
            Timber.d("delete ${_plan.value}")
        }
        return isSuccess
    }

    private fun observePlan() {
        viewModelScope.launch {
            plan.collect { plan ->
                _dayPlanList.value = plan?.datesDetail
            }
        }
    }
}
