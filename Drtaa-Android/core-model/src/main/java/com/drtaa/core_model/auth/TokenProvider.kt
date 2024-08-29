package com.drtaa.core_model.auth

interface TokenProvider {
    suspend fun getAccessToken(): String
}