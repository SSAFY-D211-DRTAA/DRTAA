package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.datasource.TokenDataSource
import com.drtaa.core_data.repository.TokenRepository
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import javax.inject.Inject

class TokenRepositoryImpl @Inject constructor(
    private val tokenDataSource: TokenDataSource
) : TokenRepository {
    override suspend fun getAccessToken(): String = tokenDataSource.getAccessToken()

    override suspend fun setAccessToken(accessToken: String) =
        tokenDataSource.setAccessToken(accessToken)

    override suspend fun clearToken(): Flow<Result<Unit>> = flow {
        tokenDataSource.clearToken().onSuccess {
            emit(Result.success(Unit))
        }.onFailure {
            emit(Result.failure(it))
        }
    }

    override suspend fun setRefreshToken(refreshToken: String) {
        tokenDataSource.setRefreshToken(refreshToken)
    }

    override suspend fun getRefreshToken(): String = tokenDataSource.getRefreshToken()
}