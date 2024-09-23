package com.drtaa.feature_tour.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import androidx.paging.PagingData
import androidx.paging.cachedIn
import com.drtaa.core_data.repository.TourRepository
import com.drtaa.core_model.tour.TourItem
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class TourViewModel @Inject constructor(
    private val tourRepository: TourRepository
) : ViewModel() {
    private val _pagedTour = MutableStateFlow<PagingData<TourItem>>(PagingData.empty())
    val pagedTour: StateFlow<PagingData<TourItem>>
        get() = _pagedTour

    fun getLocationBasedList(mapX: String, mapY: String, radius: String) {
        viewModelScope.launch {
            tourRepository.getLocationBasedList(mapX, mapY, radius).cachedIn(viewModelScope)
                .collect {
                    Timber.tag("pager").d("")
                    _pagedTour.value = it
                }
        }
    }
}