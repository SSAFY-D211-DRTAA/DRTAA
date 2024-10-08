package com.drtaa.feature_tour.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import androidx.paging.PagingData
import androidx.paging.cachedIn
import com.drtaa.core_data.repository.PlanRepository
import com.drtaa.core_data.repository.TourRepository
import com.drtaa.core_model.plan.PlanItem
import com.drtaa.core_model.tour.TourItem
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class TourViewModel @Inject constructor(
    private val tourRepository: TourRepository,
    private val planRepository: PlanRepository
) : ViewModel() {
    private val _pagedTour = MutableStateFlow<PagingData<TourItem>?>(PagingData.empty())
    val pagedTour: StateFlow<PagingData<TourItem>?>
        get() = _pagedTour

    private val _planList = MutableStateFlow<List<PlanItem>?>(emptyList())
    val planList: StateFlow<List<PlanItem>?> = _planList

    init {
        getPlanList()
    }

    fun getPlanList() {
        viewModelScope.launch {
            planRepository.getTodayPlanList().collect { result ->
                result.onSuccess { planList ->
                    _planList.value = planList
                }.onFailure {
                    _planList.value = null
                }
            }
        }
    }

    fun getLocationBasedList(mapX: String, mapY: String, radius: String) {
        viewModelScope.launch {
            val location = "$mapX $mapY"
            tourRepository.getLocationBasedList(DEFAULT_LNG, DEFAULT_LAT, radius)
                .collect { result ->
                    result.onSuccess { pagingDataFlow ->
                        pagingDataFlow
                            .cachedIn(viewModelScope)
                            .collect {
                                Timber.tag("pager").d(location)
                                _pagedTour.value = it
                            }
                    }.onFailure {
                        _pagedTour.value = null
                    }
                }
        }
    }

    companion object {
        const val DEFAULT_LAT = "37.5749543"
        const val DEFAULT_LNG = "126.886716"
    }
}