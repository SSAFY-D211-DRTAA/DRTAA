package com.drtaa.core_data.repository

import com.drtaa.core_model.data.Tokens
import com.drtaa.core_model.data.User
import kotlinx.coroutines.flow.Flow

interface SignRepository {
    suspend fun getTokens(user: User): Flow<Result<Tokens>>
}