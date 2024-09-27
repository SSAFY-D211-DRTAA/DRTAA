package com.drtaa.feature_plan.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.NaverRepository
import com.drtaa.core_data.repository.PlanRepository
import com.drtaa.core_model.map.Search
import com.drtaa.core_model.plan.PlanSimple
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
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

    init {
        getPlanList()
    }

    private fun getPlanList() {
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
}