package com.drtaa.core_model.network

data class ResponseLogin(
    val accessToken: String = "",
    val grantType: String = "",
    val refreshToken: String = "",
)