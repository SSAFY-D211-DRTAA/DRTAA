package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.datasource.TokenDataSource
import com.drtaa.core_data.repository.TokenRepository
import javax.inject.Inject

class TokenRepositoryImpl @Inject constructor(
    private val tokenDataSource: TokenDataSource
) : TokenRepository {
    override suspend fun getAccessToken(): String = tokenDataSource.getAccessToken()

    override suspend fun setAccessToken(accessToken: String) =
        tokenDataSource.setAccessToken(accessToken)

    override suspend fun clearToken() = tokenDataSource.clearToken()
}