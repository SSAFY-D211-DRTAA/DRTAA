package com.drtaa.core_model.network

data class RequestLogin(
    val userLogin: String,
    val userNickname: String,
    val userProviderId: String
)