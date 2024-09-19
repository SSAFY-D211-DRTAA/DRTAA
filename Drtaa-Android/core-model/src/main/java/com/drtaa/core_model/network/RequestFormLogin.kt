package com.drtaa.core_model.network

import com.drtaa.core_model.sign.UserLoginInfo

data class RequestFormLogin(
    val userLogin: String,
    val userPassword: String,
    val userProviderId: String
) : UserLoginInfo