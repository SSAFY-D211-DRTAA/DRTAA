package com.drtaa.feature_home.viewmodel

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
class HomeViewModel @Inject constructor(
    private val signRepository: SignRepository
) : ViewModel() {

    private val _currentUser = MutableStateFlow<SocialUser?>(null)
    val currentUser: StateFlow<SocialUser?> = _currentUser

    init {
        getUserData()
    }

    fun getUserData() {
        viewModelScope.launch {
            signRepository.getUserData().collect() { result ->
                result.onSuccess { user ->
                    _currentUser.value = user
                }.onFailure { error ->
                    Timber.e("유저 정보 조회 오류")
                }
            }
        }
    }
}