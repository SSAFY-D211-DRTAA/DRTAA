package com.drtaa.feature_home.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.bumptech.glide.Glide.init
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_model.sign.SocialUser
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class HomeViewModel @Inject constructor(
    private val signRepository: SignRepository
) : ViewModel() {

    private val _currentUser = MutableStateFlow<SocialUser?>(null)
    val currentUser: StateFlow<SocialUser?> = _currentUser

    private val _refreshTrigger = MutableStateFlow(-1)
    private var cnt = 0

    init {
        cnt = 0
        viewModelScope.launch {
            _refreshTrigger.combine(signRepository.getUserData()) { _, userResult ->
                userResult
            }.collect { result ->
                result.onSuccess { user ->
                    _currentUser.value = user
                    Timber.d("성공성공성공")
                }.onFailure { error ->
                    Timber.e("유저 정보 조회 오류: ${error.message}")
                }
            }
        }
    }

    fun refreshUserData() {
        _refreshTrigger.value = cnt++
        Timber.d("갱신")
    }
}