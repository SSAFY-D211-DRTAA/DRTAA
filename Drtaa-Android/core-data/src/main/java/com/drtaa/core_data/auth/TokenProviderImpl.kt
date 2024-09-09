package com.drtaa.core_data.auth

import com.drtaa.core_data.repository.TokenRepository
import com.drtaa.core_model.auth.TokenProvider
import javax.inject.Inject

class TokenProviderImpl @Inject constructor(
    private val tokenRepository: TokenRepository
) : TokenProvider {
    override suspend fun getAccessToken(): String {
        return tokenRepository.getAccessToken()
    }
}