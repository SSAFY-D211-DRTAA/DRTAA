package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.datasource.SignDataSource
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_data.util.ResultWrapper
import com.drtaa.core_data.util.safeApiCall
import com.drtaa.core_model.data.Tokens
import com.drtaa.core_model.data.User
import com.drtaa.core_model.data.toEntity
import com.drtaa.core_model.data.toRequestLogin
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import timber.log.Timber
import javax.inject.Inject

class SignRepositoryImpl @Inject constructor(
    private val signDataSource: SignDataSource
) : SignRepository {
    override suspend fun getTokens(user: User): Flow<Result<Tokens>> = flow {
        when (val response = safeApiCall {
            signDataSource.getTokens(user.toRequestLogin())
        }) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data.toEntity()))
                Timber.d("성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.d("실패")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 에러")))
                Timber.d("네트워크 에러")
            }

        }

    }
}