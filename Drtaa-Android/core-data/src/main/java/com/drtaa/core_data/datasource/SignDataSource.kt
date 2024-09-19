package com.drtaa.core_data.datasource

import com.drtaa.core_model.sign.UserLoginInfo
import com.drtaa.core_model.network.ResponseLogin
import okhttp3.MultipartBody
import okhttp3.RequestBody

interface SignDataSource {
    suspend fun getTokens(userLoginInfo: UserLoginInfo): ResponseLogin
    suspend fun signUp(requestSignUp: RequestBody, image: MultipartBody.Part?): String
    suspend fun checkDuplicatedId(userProviderId: String): Boolean
}