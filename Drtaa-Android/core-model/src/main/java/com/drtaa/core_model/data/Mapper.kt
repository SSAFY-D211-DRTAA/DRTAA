package com.drtaa.core_model.data

import com.drtaa.core_model.network.RequestLogin
import com.drtaa.core_model.network.ResponseLogin

fun ResponseLogin.toEntity(): Tokens {
    return Tokens(
        accessToken = this.accessToken,
        refreshToken = this.refreshToken
    )
}

fun User.toRequestLogin(): RequestLogin {
    return RequestLogin(
        userLogin = this.userLogin!!,
        userProviderId = this.id!!,
        userNickname = this.nickname!!
    )
}