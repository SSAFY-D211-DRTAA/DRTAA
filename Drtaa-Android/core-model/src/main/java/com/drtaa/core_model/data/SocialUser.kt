package com.drtaa.core_model.data

data class SocialUser(
    val userLogin: String,
    val id: String,
    val name: String? = null,
    val nickname: String,
    val profileImageUrl: String? = null,
    val accessToken: String? = null,
    val refreshToken: String? = null,
) : UserLoginInfo
