package com.drtaa.core_data.datasourceimpl

import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.stringPreferencesKey
import com.drtaa.core_data.datasource.SignDataSource
import com.drtaa.core_model.sign.SocialUser
import com.drtaa.core_model.sign.UserLoginInfo
import com.drtaa.core_model.util.toRequestLogin
import com.drtaa.core_model.network.RequestFormLogin
import com.drtaa.core_model.network.ResponseLogin
import com.drtaa.core_network.api.SignAPI
import com.drtaa.core_network.di.Auth
import com.drtaa.core_network.di.NoAuth
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.map
import okhttp3.MultipartBody
import okhttp3.RequestBody
import javax.inject.Inject
import javax.inject.Named

class SignDataSourceImpl @Inject constructor(
    @NoAuth
    private val signAPI: SignAPI,

    @Auth
    private val signAPIAuth: SignAPI,

    @Named("USER_DATASTORE")
    private val dataStore: DataStore<Preferences>
) : SignDataSource {
    override suspend fun getTokens(userLoginInfo: UserLoginInfo): ResponseLogin {
        return when (userLoginInfo) {
            is SocialUser -> signAPI.socialLogin(userLoginInfo.toRequestLogin())
            else -> signAPI.formLogin(userLoginInfo as RequestFormLogin)
        }
    }

    override suspend fun signUp(requestSignUp: RequestBody, image: MultipartBody.Part?): String {
        return signAPI.signUp(requestSignUp, image)
    }

    override suspend fun checkDuplicatedId(userProviderId: String): Boolean {
        return signAPI.checkDuplicatedId(userProviderId)
    }

    override suspend fun getUserData(): SocialUser {
        return dataStore.data.map { prefs ->
            SocialUser(
                prefs[USER_LOGIN_TYPE] ?: "",
                prefs[USER_ID] ?: "",
                prefs[USER_NAME] ?: "",
                prefs[USER_NICKNAME] ?: "",
                prefs[USER_PROFILE_IMAGE] ?: "",
            )
        }.first()
    }

    override suspend fun setUserData(user: SocialUser) {
        dataStore.edit { preferences ->
            preferences[USER_LOGIN_TYPE] = user.userLogin
            preferences[USER_ID] = user.id
            preferences[USER_NAME] = user.name ?: ""
            preferences[USER_NICKNAME] = user.nickname
            preferences[USER_PROFILE_IMAGE] = user.profileImageUrl ?: ""
        }
    }

    override suspend fun clearUserData() {
        dataStore.edit { preferences ->
            preferences.clear()
        }
    }

    override suspend fun updateUserProfileImage(image: MultipartBody.Part?): String {
        return signAPIAuth.updateUserProfileImage(image)
    }

    companion object {
        private val USER_ID = stringPreferencesKey("user_id")
        private val USER_NAME = stringPreferencesKey("user_name")
        private val USER_NICKNAME = stringPreferencesKey("user_nickname")
        private val USER_PROFILE_IMAGE = stringPreferencesKey("user_profile_image")
        private val USER_LOGIN_TYPE = stringPreferencesKey("user_login_type")
    }
}