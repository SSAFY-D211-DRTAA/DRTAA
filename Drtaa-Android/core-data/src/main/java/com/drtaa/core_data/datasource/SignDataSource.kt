package com.drtaa.core_data.datasource

import com.drtaa.core_model.data.UserLoginInfo
import com.drtaa.core_model.network.RequestFormLogin
import com.drtaa.core_model.network.RequestSignUp
import com.drtaa.core_model.network.RequestSocialLogin
import com.drtaa.core_model.network.ResponseLogin
import okhttp3.MultipartBody

interface SignDataSource {
    suspend fun getTokens(userLoginInfo: UserLoginInfo): ResponseLogin
    suspend fun signUp(requestSignUp: RequestSignUp, image: MultipartBody.Part?): String
}