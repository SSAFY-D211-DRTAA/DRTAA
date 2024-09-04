package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.datasource.SignDataSource
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_data.util.ResultWrapper
import com.drtaa.core_data.util.safeApiCall
import com.drtaa.core_model.data.SocialUser
import com.drtaa.core_model.data.Tokens
import com.drtaa.core_model.data.UserLoginInfo
import com.drtaa.core_model.data.toRequestLogin
import com.drtaa.core_model.data.toTokens
import com.drtaa.core_model.network.RequestFormLogin
import com.drtaa.core_model.network.RequestSignUp
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import okhttp3.MultipartBody
import timber.log.Timber
import javax.inject.Inject

class SignRepositoryImpl @Inject constructor(
    private val signDataSource: SignDataSource
) : SignRepository {
    override suspend fun getTokens(userLoginInfo: UserLoginInfo): Flow<Result<Tokens>> = flow {
        when (val response = safeApiCall {
            signDataSource.getTokens(userLoginInfo)
        }) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data.toTokens()))
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

    override suspend fun signUp(
        requestSignUp: RequestSignUp,
        image: MultipartBody.Part?
    ): Flow<Result<String>> = flow {
        when (val response = safeApiCall {
            signDataSource.signUp(requestSignUp, image)
        }) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
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