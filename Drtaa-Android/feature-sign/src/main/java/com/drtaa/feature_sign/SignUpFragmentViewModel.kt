package com.drtaa.feature_sign

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_model.network.RequestSignUp
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class SignUpFragmentViewModel @Inject constructor(
    private val signRepository: SignRepository
) : ViewModel() {

    private val _isSignUpSuccess = MutableStateFlow(false)
    val isSignUpSuccess: StateFlow<Boolean> = _isSignUpSuccess


    fun signUp(id: String, pw: String, nickname: String) {
        val requestSignUp = RequestSignUp(
            userProviderId = id,
            userPassword = pw,
            userNickname = nickname,
            userIsAdmin = false
        )
        viewModelScope.launch {
            signRepository.signUp(requestSignUp, null).collect { result ->
                result.onSuccess {
                    _isSignUpSuccess.emit(true)
                }.onFailure {
                    _isSignUpSuccess.emit(false)
                }

            }
        }
    }
}