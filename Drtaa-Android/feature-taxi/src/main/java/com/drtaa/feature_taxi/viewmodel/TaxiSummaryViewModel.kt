package com.drtaa.feature_taxi.viewmodel

import androidx.lifecycle.ViewModel
import com.drtaa.core_data.repository.TaxiRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class TaxiSummaryViewModel @Inject constructor(
    private val taxiRepository: TaxiRepository
) : ViewModel(){

}