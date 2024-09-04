package com.drtaa.core_network.api

import com.drtaa.core_model.network.RequestFormLogin
import com.drtaa.core_model.network.RequestSocialLogin
import com.drtaa.core_model.network.ResponseLogin
import retrofit2.http.Body
import retrofit2.http.POST

interface SignAPI {
    @POST("user/login/social")
    suspend fun socialLogin(
        @Body requestSocialLogin: RequestSocialLogin
    ): ResponseLogin

    @POST("user/login/form")
    suspend fun formLogin(
        @Body requestFormLogin: RequestFormLogin
    ): ResponseLogin
}