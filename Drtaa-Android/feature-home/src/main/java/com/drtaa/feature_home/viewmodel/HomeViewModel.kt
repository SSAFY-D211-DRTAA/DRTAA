package com.drtaa.feature_home.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_model.sign.SocialUser
import com.drtaa.core_model.util.Status
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class HomeViewModel @Inject constructor(
    private val signRepository: SignRepository,
    private val rentRepository: RentRepository
) : ViewModel() {

    private val _currentUser = MutableStateFlow<SocialUser?>(null)
    val currentUser: StateFlow<SocialUser?> = _currentUser

    private val _rentStatus = MutableStateFlow<String>("")
    val rentStatus: StateFlow<String> = _rentStatus

    fun refreshUserData() {
        viewModelScope.launch {
            signRepository.getUserData().collect { result ->
                result.onSuccess { user ->
                    _currentUser.value = user
                }.onFailure { error ->
                    Timber.e("로그인 유저 정보 조회 오류: ${error.message}")
                }
            }
        }
    }

    fun getRentStatus() {
        viewModelScope.launch {
            rentRepository.getRentStatus().collect { result ->
                result.onSuccess { status ->
                    _rentStatus.value = when (status.rentStatus) {
                        Status.RESERVED.status -> "이용 예정 차량이 있습니다\n차량을 호출해 보세요!"
                        Status.IN_PROGRESS.status -> {
                            when (status.rentCarDrivingStatus) {
                                Status.IDLING.status -> "이용 예정 차량이 있습니다\n차량을 호출해 보세요!"
                                Status.CALLING.status -> "현재 차량을 호출 중입니다!"
                                Status.DRIVING.status -> "현재 차량이 주행 중입니다!"
                                Status.PARKING.status -> "현재 차량이 주차 중입니다!"
                                Status.WAITING.status -> "현재 차량이 대기 중입니다!"
                                Status.CHARGING.status -> "현재 차량을 충전 중입니다!"
                                else -> ""
                            }
                        }

                        else -> "이용 예정인 차량이 없습니다\n렌트를 예약해 보세요!"
                    }
                }.onFailure {
                    _rentStatus.value = "오류가 발생했습니다.\n다시 시도해주세요!"
                }
            }
        }
    }
}