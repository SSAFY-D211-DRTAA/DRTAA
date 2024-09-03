package com.drtaa.core_data.datasource

import com.drtaa.core_model.network.RequestLogin
import com.drtaa.core_model.network.ResponseLogin

interface SignDataSource {
    suspend fun getTokens(requestLogin: RequestLogin): ResponseLogin
}