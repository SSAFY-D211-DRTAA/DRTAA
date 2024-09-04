package com.drtaa.feature_sign

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_data.repository.TokenRepository
import com.drtaa.core_model.data.Tokens
import com.drtaa.core_model.data.User
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class SignViewModel @Inject constructor(
    private val tokenRepository: TokenRepository,
    private val signRepository: SignRepository,
) : ViewModel() {

    private val _tokens = MutableSharedFlow<Result<Tokens>>()
    val tokens: SharedFlow<Result<Tokens>> = _tokens

    private fun setTokens(tokens: Tokens) {
        viewModelScope.launch {
            tokenRepository.setAccessToken(tokens.accessToken)
        }
    }

    fun getTokens(user: User) {
        viewModelScope.launch {
            signRepository.getTokens(user).collect { result ->
                result.onSuccess { data ->
                    Timber.tag("tokens").d("success $data")
                    _tokens.emit(Result.success(data))
                    setTokens(data)
                }.onFailure {
                    Timber.tag("tokens").d("fail")
                    _tokens.emit(Result.failure(Exception("fail")))
                }
            }
        }
    }
}