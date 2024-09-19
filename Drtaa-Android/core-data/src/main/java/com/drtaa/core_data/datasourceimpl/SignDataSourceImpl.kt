package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.SignDataSource
import com.drtaa.core_model.sign.SocialUser
import com.drtaa.core_model.sign.UserLoginInfo
import com.drtaa.core_model.util.toRequestLogin
import com.drtaa.core_model.network.RequestFormLogin
import com.drtaa.core_model.network.ResponseLogin
import com.drtaa.core_network.api.SignAPI
import okhttp3.MultipartBody
import okhttp3.RequestBody
import javax.inject.Inject

class SignDataSourceImpl @Inject constructor(
    private val signAPI: SignAPI
) : SignDataSource {
    override suspend fun getTokens(userLoginInfo: UserLoginInfo): ResponseLogin {
        return when (userLoginInfo) {
            is SocialUser -> signAPI.socialLogin(userLoginInfo.toRequestLogin())
            else -> signAPI.formLogin(userLoginInfo as RequestFormLogin)
        }
    }

    override suspend fun signUp(requestSignUp: RequestBody, image: MultipartBody.Part?): String {
        return signAPI.signUp(requestSignUp, image)
    }

    override suspend fun checkDuplicatedId(userProviderId: String): Boolean {
        return signAPI.checkDuplicatedId(userProviderId)
    }
}