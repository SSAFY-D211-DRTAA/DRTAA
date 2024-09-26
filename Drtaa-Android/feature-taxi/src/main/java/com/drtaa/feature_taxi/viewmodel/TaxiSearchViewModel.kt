package com.drtaa.feature_taxi.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.NaverRepository
import com.drtaa.core_model.map.Search
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject


@HiltViewModel
class TaxiSearchViewModel @Inject constructor(
    private val naverRepository: NaverRepository
) : ViewModel() {
    private val _searchList = MutableSharedFlow<Result<List<Search>>>()
    val searchList: SharedFlow<Result<List<Search>>> = _searchList

    private val _selectedSearchItem = MutableStateFlow<Search?>(null)
    val selectedSearchItem: StateFlow<Search?> = _selectedSearchItem

    fun setSelectedSearchItem(search: Search) {
        viewModelScope.launch {
            _selectedSearchItem.emit(search)
        }
    }

    fun getSearchList(keyword: String) {
        viewModelScope.launch {
            naverRepository.getSearchList(keyword).collect { result ->
                result.onSuccess { data ->
                    Timber.tag("search").d("성공 $data")
                    _searchList.emit(Result.success(data))
                }.onFailure {
                    Timber.tag("search").d("실패!")
                    _searchList.emit(Result.failure(Exception("실패")))
                }
            }
        }
    }
}