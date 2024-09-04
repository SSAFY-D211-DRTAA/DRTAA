package com.drtaa.core_data.repository

import com.drtaa.core_model.data.Tokens
import com.drtaa.core_model.data.UserLoginInfo
import com.drtaa.core_model.network.RequestSignUp
import kotlinx.coroutines.flow.Flow
import okhttp3.MultipartBody
import okhttp3.RequestBody
import java.io.File

interface SignRepository {
    suspend fun getTokens(userLoginInfo: UserLoginInfo): Flow<Result<Tokens>>
    suspend fun signUp(
        requestSignUp: RequestSignUp,
        image: File?
    ): Flow<Result<String>>
}