package com.drtaa.core_network.api

import com.drtaa.core_model.network.RequestFormLogin
import com.drtaa.core_model.network.RequestSocialLogin
import com.drtaa.core_model.network.ResponseLogin
import com.drtaa.core_model.sign.RequestFCMToken
import com.drtaa.core_model.sign.ResponseUserInfo
import okhttp3.MultipartBody
import okhttp3.RequestBody
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.Multipart
import retrofit2.http.PATCH
import retrofit2.http.POST
import retrofit2.http.Part
import retrofit2.http.Path
import retrofit2.http.Query

interface SignAPI {
    @Multipart
    @POST("user/signup")
    suspend fun signUp(
        @Part("formSignUpRequestDTO") requestSignUp: RequestBody,
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

    @GET("user/signup/{userProviderId}")
    suspend fun checkDuplicatedId(
        @Path("userProviderId") userProviderId: String
    ): Boolean

    @Multipart
    @PATCH("user/img")
    suspend fun updateUserProfileImage(
        @Part image: MultipartBody.Part?
    ): String

    @POST("user/fcm-token")
    suspend fun setFCMToken(
        @Body request: RequestFCMToken
    ): String

    @GET("user/info")
    suspend fun getUserInfo(): ResponseUserInfo

    @POST("user/jwt-token")
    suspend fun getNewAccessToken(
        @Query("userRefreshToken") refreshToken: String
    ): ResponseLogin
}