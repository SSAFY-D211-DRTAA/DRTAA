package com.drtaa.feature_taxi.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.NaverRepository
import com.drtaa.core_model.map.Search
import com.drtaa.core_model.network.ResponseReverseGeocode
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

    private val _reverseGeocode = MutableStateFlow<Result<String>?>(null)
    val reverseGeocode: StateFlow<Result<String>?> = _reverseGeocode

    fun setSelectedSearchItem(search: Search) {
        _selectedSearchItem.value = search
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

    fun getReverseGeocode(latitude: Double, longitude: Double) {
        viewModelScope.launch {
            naverRepository.getReverseGeocode(latitude, longitude).collect { result ->
                result.onSuccess { data ->
                    _reverseGeocode.emit(Result.success(formatAddress(data)))
                }.onFailure {
                    _reverseGeocode.emit(Result.failure(Exception("fail")))
                }
            }
        }
    }

    private fun formatAddress(response: ResponseReverseGeocode): String {
        val result = response.results.firstOrNull()
        return if (result != null) {
            val region = result.region
            val land = result.land
            "${region.area1.name} ${region.area2.name} ${region.area3.name} ${land?.addition0?.value}"
        } else {
            "주소를 찾을 수 없습니다."
        }
    }
}