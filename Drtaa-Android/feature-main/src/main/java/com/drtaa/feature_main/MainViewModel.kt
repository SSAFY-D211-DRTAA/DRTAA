package com.drtaa.feature_main

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.PlanRepository
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_data.repository.TokenRepository
import com.drtaa.core_data.repository.TravelRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class MainViewModel @Inject constructor(
    private val signRepository: SignRepository,
    private val planRepository: PlanRepository
) : ViewModel() {
    fun setFCMToken(fcmToken: String) {
        viewModelScope.launch {
            signRepository.setFCMToken(fcmToken).collect { result ->
                result.onSuccess {
                    Timber.tag("fcm").d("FCM 토큰 저장 성공 $it")
                }.onFailure {
                    Timber.tag("fcm").d("FCM 토큰 저장 실패 $it")
                }
            }
        }
    }
}