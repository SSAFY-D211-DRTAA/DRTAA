package com.drtaa.feature_home.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.MapRepository
import com.drtaa.core_model.network.ResponseSearch
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class HomeViewModel @Inject constructor(
    private val mapRepository: MapRepository
) : ViewModel() {
    private val _searchList = MutableSharedFlow<Result<ResponseSearch>>()
    val searchList: SharedFlow<Result<ResponseSearch>> = _searchList

    fun getSearchList(keyword: String) {
        viewModelScope.launch {
            mapRepository.getSearchList(keyword).collect { result ->
                result.onSuccess { data ->
                    Timber.tag("tokens").d("success $data")
                    _searchList.emit(Result.success(data))
                }.onFailure {
                    Timber.tag("tokens").d("fail")
                    _searchList.emit(Result.failure(Exception("fail")))
                }
            }
        }
    }
}