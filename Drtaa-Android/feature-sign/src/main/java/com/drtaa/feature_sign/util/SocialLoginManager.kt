package com.drtaa.feature_sign.util

import android.content.Context
import android.content.Intent
import android.provider.Settings
import androidx.credentials.ClearCredentialStateRequest
import androidx.credentials.CredentialManager
import androidx.credentials.CustomCredential
import androidx.credentials.GetCredentialRequest
import androidx.credentials.GetCredentialResponse
import androidx.credentials.exceptions.GetCredentialException
import com.drtaa.core_model.data.SocialUser
import com.drtaa.core_model.util.Social
import com.google.android.libraries.identity.googleid.GetGoogleIdOption
import com.google.android.libraries.identity.googleid.GoogleIdTokenCredential
import com.google.android.libraries.identity.googleid.GoogleIdTokenParsingException
import com.navercorp.nid.NaverIdLoginSDK
import com.navercorp.nid.oauth.NidOAuthLogin
import com.navercorp.nid.oauth.OAuthLoginCallback
import com.navercorp.nid.profile.NidProfileCallback
import com.navercorp.nid.profile.data.NidProfileResponse
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject
import javax.inject.Singleton

@Singleton
class SocialLoginManager @Inject constructor(
    private val googleIdOption: GetGoogleIdOption,
) {
    private val _resultLogin = MutableSharedFlow<Result<SocialUser>>()
    val resultLogin: SharedFlow<Result<SocialUser>> = _resultLogin

    private fun signScope(action: suspend () -> Unit) {
        CoroutineScope(Dispatchers.IO).launch {
            action()
        }
    }

    private val profileCallback = object : NidProfileCallback<NidProfileResponse> {
        override fun onSuccess(result: NidProfileResponse) {
            signScope {
                _resultLogin.emit(
                    Result.success(
                        SocialUser(
                            userLogin = Social.NAVER.type,
                            id = result.profile?.id.orEmpty(),
                            name = result.profile?.name,
                            nickname = result.profile?.nickname.orEmpty(),
                            profileImageUrl = result.profile?.profileImage,
                            accessToken = NaverIdLoginSDK.getAccessToken(),
                            refreshToken = NaverIdLoginSDK.getRefreshToken()
                        )
                    )
                )
            }
        }

        override fun onFailure(httpStatus: Int, message: String) {
            signScope {
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
            signScope {
                NidOAuthLogin().callProfileApi(profileCallback)
            }
        }

        override fun onFailure(httpStatus: Int, message: String) {
            signScope {
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

    fun login(socialType: String, context: Context) {
        when (socialType) {
            Social.NAVER.type -> NaverIdLoginSDK.authenticate(context, oAuthLoginCallback)
            Social.GOOGLE.type -> {
                signScope {
                    googleLogin(context)
                }
            }
        }
    }

    fun logout(socialType: String, context: Context) {
        when (socialType) {
            Social.NAVER.type -> NaverIdLoginSDK.logout()
            Social.GOOGLE.type -> signScope {
                val credentialManager = CredentialManager.create(context)
                credentialManager.clearCredentialState(request = ClearCredentialStateRequest())
            }
        }
    }

    private suspend fun googleLogin(context: Context) {
        val credentialManager = CredentialManager.create(context)

        val request: GetCredentialRequest = GetCredentialRequest.Builder()
            .addCredentialOption(googleIdOption)
            .build()

        coroutineScope {
            try {
                val result = credentialManager.getCredential(
                    request = request,
                    context = context
                )
                handleSignIn(result)
            } catch (e: GetCredentialException) {
                if (e.type == android.credentials.GetCredentialException.TYPE_NO_CREDENTIAL) {
                    context.startActivity(Intent(Settings.ACTION_ADD_ACCOUNT))
                }
            }
        }
    }

    private suspend fun handleSignIn(result: GetCredentialResponse) {
        when (val credential = result.credential) {
            is CustomCredential -> {
                if (credential.type == GoogleIdTokenCredential.TYPE_GOOGLE_ID_TOKEN_CREDENTIAL) {
                    try {
                        val googleIdTokenCredential = GoogleIdTokenCredential
                            .createFrom(credential.data)

                        _resultLogin.emit(
                            Result.success(
                                SocialUser(
                                    userLogin = Social.GOOGLE.type,
                                    id = googleIdTokenCredential.id,
                                    name = googleIdTokenCredential.displayName,
                                    nickname = googleIdTokenCredential.displayName.orEmpty(),
                                    profileImageUrl = googleIdTokenCredential.profilePictureUri.toString()
                                )
                            )
                        )
                    } catch (e: GoogleIdTokenParsingException) {
                        Timber.d("Received an invalid google id token response", e)
                        _resultLogin.emit(
                            Result.failure(
                                Exception("${e.message}")
                            )
                        )
                    }
                } else {
                    Timber.d("Unexpected type of credential")
                    _resultLogin.emit(
                        Result.failure(
                            Exception("Unexpected type of credential")
                        )
                    )
                }
            }
        }
    }
}