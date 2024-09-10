package com.drtaa.core_model.network

data class RequestSignUp(
    val userIsAdmin: Boolean = false,
    val userNickname: String,
    val userPassword: String,
    val userProviderId: String
)