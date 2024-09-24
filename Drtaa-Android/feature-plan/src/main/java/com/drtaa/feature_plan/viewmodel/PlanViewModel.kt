package com.drtaa.feature_plan.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_model.plan.Plan
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class PlanViewModel @Inject constructor(

) : ViewModel() {
    private val _plan = MutableStateFlow<Plan?>(null)
    val plan: StateFlow<Plan?> = _plan

    private val _isEditMode = MutableStateFlow(false)
    val isEditMode: StateFlow<Boolean> = _isEditMode

    init {
        getPlan()
    }

    private fun getPlan() {
        viewModelScope.launch {
            _plan.emit(seoulTrip)
        }
    }

    fun setEditMode(isEditMode: Boolean) {
        _isEditMode.value = isEditMode
    }

    companion object {
        val seoulTrip = Plan(
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
    }
}