package com.drtaa.feature_sign.util

import android.content.Context
import com.drtaa.core_model.data.SocialUser
import com.navercorp.nid.NaverIdLoginSDK
import com.navercorp.nid.oauth.NidOAuthLogin
import com.navercorp.nid.oauth.OAuthLoginCallback
import com.navercorp.nid.profile.NidProfileCallback
import com.navercorp.nid.profile.data.NidProfileResponse
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.launch

object NaverLoginManager {
    private val _resultLogin = MutableSharedFlow<Result<SocialUser>>()
    val resultLogin: SharedFlow<Result<SocialUser>> = _resultLogin

    private fun loginScope(action: suspend () -> Unit) {
        CoroutineScope(Dispatchers.IO).launch {
            action()
        }
    }

    private val profileCallback = object : NidProfileCallback<NidProfileResponse> {
        override fun onSuccess(result: NidProfileResponse) {
            loginScope {
                _resultLogin.emit(
                    Result.success(
                        SocialUser(
                            userLogin = "Naver",
                            id = result.profile?.id,
                            name = result.profile?.name,
                            nickname = result.profile?.nickname,
                            profileImageUrl = result.profile?.profileImage,
                            accessToken = NaverIdLoginSDK.getAccessToken(),
                            refreshToken = NaverIdLoginSDK.getRefreshToken()
                        )
                    )
                )
            }

        }

        override fun onFailure(httpStatus: Int, message: String) {
            loginScope {
                _resultLogin.emit(
                    Result.failure(
                        Exception("code: $httpStatus \n message: $message")
                    )
                )
            }
        }

        override fun onError(errorCode: Int, message: String) {
            onFailure(errorCode, message)
        }
    }

    private val oAuthLoginCallback = object : OAuthLoginCallback {
        override fun onSuccess() {
            loginScope {
                NidOAuthLogin().callProfileApi(profileCallback)
            }
        }

        override fun onFailure(httpStatus: Int, message: String) {
            loginScope {
                _resultLogin.emit(
                    Result.failure(
                        Exception("code: $httpStatus \n message: $message")
                    )
                )
            }
        }

        override fun onError(errorCode: Int, message: String) {
            onFailure(errorCode, message)
        }
    }

    fun login(context: Context) {
        NaverIdLoginSDK.authenticate(context, oAuthLoginCallback)
    }

    fun logout() {
        NaverIdLoginSDK.logout()
    }
}