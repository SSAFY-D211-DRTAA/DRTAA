package com.drtaa.core_model.sign

data class ResponseUserInfo(
    val userId: Int,
    val userIsAdmin: Boolean,
    val userLogin: String,
    val userNickname: String,
    val userProfileImg: String,
    val userSiginupDate: String
)