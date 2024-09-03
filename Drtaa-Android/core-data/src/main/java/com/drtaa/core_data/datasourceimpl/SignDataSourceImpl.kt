package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.SignDataSource
import com.drtaa.core_model.network.RequestLogin
import com.drtaa.core_model.network.ResponseLogin
import com.drtaa.core_network.api.SignAPI
import javax.inject.Inject

class SignDataSourceImpl @Inject constructor(
    private val signAPI: SignAPI
) : SignDataSource {
    override suspend fun getTokens(requestLogin: RequestLogin): ResponseLogin {
        return signAPI.socialLogin(requestLogin)
    }

}