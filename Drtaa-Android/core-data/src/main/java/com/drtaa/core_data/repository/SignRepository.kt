package com.drtaa.core_data.repository

import com.drtaa.core_model.data.Tokens
import com.drtaa.core_model.data.SocialUser
import kotlinx.coroutines.flow.Flow

interface SignRepository {
    suspend fun getTokens(socialUser: SocialUser): Flow<Result<Tokens>>
}