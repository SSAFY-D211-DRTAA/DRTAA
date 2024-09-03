package com.drtaa.core_network.api

import com.drtaa.core_model.network.RequestLogin
import com.drtaa.core_model.network.ResponseLogin
import retrofit2.http.Body
import retrofit2.http.POST

interface SignAPI {
    @POST("/user/login/social")
    suspend fun socialLogin(
        @Body requestLogin: RequestLogin
    ): ResponseLogin
}