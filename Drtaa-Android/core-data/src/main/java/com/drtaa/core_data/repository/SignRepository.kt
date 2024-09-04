package com.drtaa.core_data.repository

import com.drtaa.core_model.data.Tokens
import com.drtaa.core_model.data.UserLoginInfo
import com.drtaa.core_model.network.RequestSignUp
import kotlinx.coroutines.flow.Flow
import okhttp3.MultipartBody

interface SignRepository {
    suspend fun getTokens(userLoginInfo: UserLoginInfo): Flow<Result<Tokens>>
    suspend fun signUp(
        requestSignUp: RequestSignUp,
        image: MultipartBody.Part?
    ): Flow<Result<String>>
}