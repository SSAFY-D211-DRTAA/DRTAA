package com.drtaa.feature_home.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_model.rent.RentSimple
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class RentHistoryViewModel @Inject constructor(
    private val rentRepository: RentRepository
) : ViewModel() {
    private val _rentHistory = MutableSharedFlow<List<RentSimple>>()
    val rentHistory: SharedFlow<List<RentSimple>> = _rentHistory

    init {
        getRentHistory()
    }

    private fun getRentHistory() {
        viewModelScope.launch {
            rentRepository.getRentHistory().collect { result ->
                result.onSuccess { data ->
                    _rentHistory.emit(descendingRentList(data))
                    Timber.d("렌트 기록 조회 성공")
                }.onFailure {
                    Timber.d("렌트 기록 조회 실패")
                }
            }
        }
    }

    private fun descendingRentList(list : List<RentSimple>): List<RentSimple> {
        return list.sortedByDescending { it.rentStartTime }
    }
}