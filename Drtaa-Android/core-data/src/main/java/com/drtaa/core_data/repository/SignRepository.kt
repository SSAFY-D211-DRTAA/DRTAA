package com.drtaa.core_data.repository

import com.drtaa.core_model.data.Tokens
import com.drtaa.core_model.sign.UserLoginInfo
import com.drtaa.core_model.network.RequestSignUp
import com.drtaa.core_model.sign.SocialUser
import kotlinx.coroutines.flow.Flow
import java.io.File

interface SignRepository {
    suspend fun getTokens(userLoginInfo: UserLoginInfo): Flow<Result<Tokens>>
    suspend fun signUp(
        requestSignUp: RequestSignUp,
        image: File?
    ): Flow<Result<String>>
    suspend fun checkDuplicatedId(userProviderId: String): Flow<Result<Boolean>>
    suspend fun getUserData(): Flow<Result<SocialUser>>
    suspend fun setUserData(user: SocialUser)
    suspend fun clearUserData()
    suspend fun updateProfileImage(image: File?): Flow<Result<String>>
}