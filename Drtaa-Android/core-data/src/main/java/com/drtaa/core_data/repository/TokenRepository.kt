package com.drtaa.core_data.repository

interface TokenRepository {
    suspend fun getAccessToken(): String
    suspend fun setAccessToken(accessToken: String)
    suspend fun clearToken()
    suspend fun setRefreshToken(refreshToken: String)
    suspend fun getRefreshToken(): String
}