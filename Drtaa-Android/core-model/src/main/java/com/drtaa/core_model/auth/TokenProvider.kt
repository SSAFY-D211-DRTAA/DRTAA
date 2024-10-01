package com.drtaa.core_model.auth

import com.drtaa.core_model.data.Tokens
import kotlinx.coroutines.flow.Flow

interface TokenProvider {
    suspend fun getAccessToken(): String
    suspend fun getRefreshToken(): String
    suspend fun getNewTokens(): Flow<Result<Tokens>>
    suspend fun getNewAccessToken(): String?
    suspend fun setAccessToken(accessToken: String)
}