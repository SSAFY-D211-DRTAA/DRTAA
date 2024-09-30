package com.drtaa.core_data.auth

import com.drtaa.core_data.repository.TokenRepository
import com.drtaa.core_data.util.ResultWrapper
import com.drtaa.core_data.util.safeApiCall
import com.drtaa.core_model.auth.Event
import com.drtaa.core_model.auth.EventBus
import com.drtaa.core_model.auth.TokenProvider
import com.drtaa.core_model.data.Tokens
import com.drtaa.core_model.util.toTokens
import com.drtaa.core_network.api.SignAPI
import com.drtaa.core_network.di.Auth
import dagger.Lazy
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import timber.log.Timber
import javax.inject.Inject

class TokenProviderImpl @Inject constructor(
    private val tokenRepository: TokenRepository,
    @Auth
    private val authSignAPI: Lazy<SignAPI>,
    private val eventBus: EventBus
) : TokenProvider {
    override suspend fun getAccessToken(): String {
        return tokenRepository.getAccessToken()
    }

    override suspend fun getRefreshToken(): String {
        return tokenRepository.getRefreshToken()
    }

    override suspend fun setAccessToken(accessToken: String) {
        return tokenRepository.setAccessToken(accessToken)
    }

    override suspend fun getNewTokens(): Flow<Result<Tokens>> = flow {
        when (
            val response = safeApiCall {
                authSignAPI.get().getNewAccessToken(tokenRepository.getRefreshToken())
            }
        ) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data.toTokens()))
                Timber.d("새 엑세스 토큰 받기 성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("새 엑세스 토큰 받기 실패")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("네트워크 에러")
            }
        }
    }

    override suspend fun getNewAccessToken(): String? {
        return try {
            var newAccessToken: String? = null
            getNewTokens().collect { result ->
                result.onSuccess { data ->
                    newAccessToken = data.accessToken
                }.onFailure {
                    newAccessToken = null
                }
            }

            if (!newAccessToken.isNullOrBlank()) {
                setAccessToken(newAccessToken!!)
            } else {
                eventBus.emitEvent(Event.LogoutEvent)
                tokenRepository.clearToken()
                return null
            }
            newAccessToken
        } catch (e: Exception) {
            Timber.d("토큰 갱신 실패: ${e.message}")
            eventBus.emitEvent(Event.LogoutEvent)
            tokenRepository.clearToken()
            null
        }
    }
}