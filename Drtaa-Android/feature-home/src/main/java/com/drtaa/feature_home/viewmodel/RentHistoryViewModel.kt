package com.drtaa.feature_home.viewmodel

import androidx.lifecycle.ViewModel
import com.drtaa.core_model.rent.RentSimple
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.SharedFlow
import javax.inject.Inject

@HiltViewModel
class RentHistoryViewModel @Inject constructor() : ViewModel() {
    private val _rentHistory = MutableSharedFlow<List<RentSimple>>()
    val rentHistory: SharedFlow<List<RentSimple>> = _rentHistory
}