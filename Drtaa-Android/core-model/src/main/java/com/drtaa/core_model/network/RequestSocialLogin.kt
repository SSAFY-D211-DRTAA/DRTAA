package com.drtaa.core_model.network

data class RequestSocialLogin(
    val userLogin: String,
    val userNickname: String,
    val userProviderId: String
)