package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.SignDataSource
import com.drtaa.core_model.data.UserLoginInfo
import com.drtaa.core_model.network.RequestFormLogin
import com.drtaa.core_model.network.RequestSignUp
import com.drtaa.core_model.network.RequestSocialLogin
import com.drtaa.core_model.network.ResponseLogin
import com.drtaa.core_network.api.SignAPI
import okhttp3.MultipartBody
import timber.log.Timber
import javax.inject.Inject

class SignDataSourceImpl @Inject constructor(
    private val signAPI: SignAPI
) : SignDataSource {
    override suspend fun getTokens(userLoginInfo: UserLoginInfo): ResponseLogin {
        return when (userLoginInfo) {
            is RequestFormLogin -> signAPI.formLogin(userLoginInfo)
            is RequestSocialLogin -> signAPI.socialLogin(userLoginInfo)
            else -> return ResponseLogin()
        }
    }

    override suspend fun signUp(requestSignUp: RequestSignUp, image: MultipartBody.Part?): String {
        return signAPI.signUp(requestSignUp, image)
    }

}