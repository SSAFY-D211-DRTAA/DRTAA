package com.drtaa.feature_payment.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_model.sign.SocialUser
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class PaymentViewModel @Inject constructor(
    private val signRepository: SignRepository
) : ViewModel() {

    private val _currentUser = MutableStateFlow<SocialUser?>(null)
    val currentUser: StateFlow<SocialUser?> = _currentUser

    init {
        currentUser()
    }

    private fun currentUser() {
        viewModelScope.launch {
            signRepository.getUserData().collect { result ->
                result.onSuccess { user ->
                    _currentUser.value = user
                }.onFailure { error ->
                    Timber.e("사용자 정보를 가져오는데 실패했습니다...")
                }
            }
        }
    }
}
