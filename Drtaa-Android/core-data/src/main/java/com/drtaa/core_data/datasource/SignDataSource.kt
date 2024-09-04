package com.drtaa.core_data.datasource

import com.drtaa.core_model.network.RequestSocialLogin
import com.drtaa.core_model.network.ResponseLogin

interface SignDataSource {
    suspend fun getTokens(requestSocialLogin: RequestSocialLogin): ResponseLogin
}