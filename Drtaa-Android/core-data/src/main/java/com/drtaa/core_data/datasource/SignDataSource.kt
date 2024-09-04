package com.drtaa.core_data.datasource

import com.drtaa.core_model.data.UserLoginInfo
import com.drtaa.core_model.network.RequestFormLogin
import com.drtaa.core_model.network.RequestSocialLogin
import com.drtaa.core_model.network.ResponseLogin

interface SignDataSource {
    suspend fun getTokens(userLoginInfo: UserLoginInfo): ResponseLogin
}