package com.drtaa.core_network.api

import com.drtaa.core_model.network.RequestFormLogin
import com.drtaa.core_model.network.RequestSignUp
import com.drtaa.core_model.network.RequestSocialLogin
import com.drtaa.core_model.network.ResponseLogin
import okhttp3.MultipartBody
import okhttp3.RequestBody
import retrofit2.http.Body
import retrofit2.http.Multipart
import retrofit2.http.POST
import retrofit2.http.Part

interface SignAPI {
    @Multipart
    @POST("user/signup")
    suspend fun signUp(
        @Part ("formSignUpRequestDTO") requestSignUp: RequestBody,
        @Part image: MultipartBody.Part?
    ): String

    @POST("user/login/social")
    suspend fun socialLogin(
        @Body requestSocialLogin: RequestSocialLogin
    ): ResponseLogin

    @POST("user/login/form")
    suspend fun formLogin(
        @Body requestFormLogin: RequestFormLogin
    ): ResponseLogin
}