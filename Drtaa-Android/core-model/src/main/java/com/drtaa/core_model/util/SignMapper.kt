package com.drtaa.core_model.util

import com.drtaa.core_model.sign.SocialUser
import com.drtaa.core_model.data.Tokens
import com.drtaa.core_model.network.RequestSocialLogin
import com.drtaa.core_model.network.ResponseLogin
import com.drtaa.core_model.sign.ResponseUserInfo

fun ResponseLogin.toTokens(): Tokens {
    return Tokens(
        accessToken = this.accessToken,
        refreshToken = this.refreshToken
    )
}

fun SocialUser.toRequestLogin(): RequestSocialLogin {
    return RequestSocialLogin(
        userLogin = this.userLogin,
        userProviderId = this.id,
        userNickname = this.nickname,
        userProfileImg = this.profileImageUrl
    )
}

fun ResponseUserInfo.toSocialUser(): SocialUser {
    return SocialUser(
        userLogin = this.userLogin,
        id = this.userId.toString(),
        nickname = this.userNickname,
        profileImageUrl = this.userProfileImg
    )
}