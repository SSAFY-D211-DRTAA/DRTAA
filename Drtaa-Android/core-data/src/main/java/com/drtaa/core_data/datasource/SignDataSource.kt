package com.drtaa.core_data.datasource

import com.drtaa.core_model.sign.UserLoginInfo
import com.drtaa.core_model.network.ResponseLogin
import com.drtaa.core_model.sign.SocialUser
import okhttp3.MultipartBody
import okhttp3.RequestBody

interface SignDataSource {
    suspend fun getTokens(userLoginInfo: UserLoginInfo): ResponseLogin
    suspend fun signUp(requestSignUp: RequestBody, image: MultipartBody.Part?): String
    suspend fun checkDuplicatedId(userProviderId: String): Boolean
    suspend fun getUserData(): SocialUser
    suspend fun setUserData(user: SocialUser)
    suspend fun clearUserData()
    suspend fun updateUserProfileImage(image: MultipartBody.Part?): SocialUser
}