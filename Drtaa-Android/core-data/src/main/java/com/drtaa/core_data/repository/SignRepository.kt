package com.drtaa.core_data.repository

import com.drtaa.core_model.data.Tokens
import com.drtaa.core_model.data.SocialUser
import com.drtaa.core_model.data.UserLoginInfo
import com.drtaa.core_model.network.RequestFormLogin
import kotlinx.coroutines.flow.Flow

interface SignRepository {
    suspend fun getTokens(userLoginInfo: UserLoginInfo): Flow<Result<Tokens>>
}