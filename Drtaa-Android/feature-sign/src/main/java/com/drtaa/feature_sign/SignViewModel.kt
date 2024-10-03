package com.drtaa.feature_sign

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_data.repository.TokenRepository
import com.drtaa.core_model.auth.TokenProvider
import com.drtaa.core_model.data.Tokens
import com.drtaa.core_model.network.RequestFormLogin
import com.drtaa.core_model.sign.UserLoginInfo
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
    private val tokenProvider: TokenProvider
) : ViewModel() {
    private val _tokens = MutableSharedFlow<Result<Tokens>>()
    val tokens: SharedFlow<Result<Tokens>> = _tokens

    fun getTokens() {
        viewModelScope.launch {
            tokenProvider.getNewTokens().collect { result ->
                result.onSuccess { data ->
                    Timber.tag("tokens").d("success $data")
                    setTokens(data)
                }.onFailure {
                    Timber.tag("tokens").d("fail")
                    _tokens.emit(Result.failure(Exception("fail")))
                    tokenRepository.clearToken()
                }
            }
        }
    }

    private fun setTokens(tokens: Tokens) {
        viewModelScope.launch {
            tokenRepository.setAccessToken(tokens.accessToken)
            tokenRepository.setRefreshToken(tokens.refreshToken)
            signRepository.getUserInfo().collect { result ->
                result.onSuccess { data ->
                    signRepository.setUserData(data)
                    Timber.tag("tokens").d("유저 정보 불러오기 및 저장 성공")
                }.onFailure {
                    Timber.tag("tokens").d("유저 정보 불러오기 및 저장 실패")
                }
            }
            _tokens.emit(Result.success(tokens))
        }
    }

    fun getTokens(userLoginInfo: UserLoginInfo) {
        viewModelScope.launch {
            signRepository.getTokens(userLoginInfo).collect { result ->
                result.onSuccess { data ->
                    Timber.tag("tokens").d("success $data")
                    setTokens(data)
                }.onFailure {
                    Timber.tag("tokens").d("fail")
                    _tokens.emit(Result.failure(Exception("fail")))
                }
            }
        }
    }

    fun formLogin(id: String, pw: String) {
        val formUser = RequestFormLogin(
            userLogin = "Form",
            userProviderId = id,
            userPassword = pw
        )
        getTokens(formUser)
    }
}