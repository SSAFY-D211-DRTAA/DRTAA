package com.drtaa.core_data.datasource

interface TokenDataSource {
    suspend fun clearToken(): Result<Unit>
    suspend fun getAccessToken(): String
    suspend fun setAccessToken(accessToken: String)
    suspend fun setRefreshToken(refreshToken: String)
    suspend fun getRefreshToken(): String
}