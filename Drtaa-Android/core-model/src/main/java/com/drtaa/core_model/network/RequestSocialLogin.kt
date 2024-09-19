package com.drtaa.core_model.network

import com.drtaa.core_model.data.UserLoginInfo

data class RequestSocialLogin(
    val userLogin: String,
    val userNickname: String,
    val userProviderId: String,
    val userProfileImg: String?
) : UserLoginInfo