package com.drtaa.core_data.datasourceimpl

import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.stringPreferencesKey
import com.drtaa.core_data.datasource.TokenDataSource
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.map
import timber.log.Timber
import javax.inject.Inject
import javax.inject.Named

class TokenDataSourceImpl @Inject constructor(
    @Named("TOKEN_DATASTORE")
    private val dataStore: DataStore<Preferences>
) : TokenDataSource {
    override suspend fun getAccessToken(): String {
        return dataStore.data.map { prefs ->
            Timber.tag("token").d("getAccessToken: ${prefs[ACCESS_TOKEN_KEY]}")
            prefs[ACCESS_TOKEN_KEY] ?: ""
        }.first()
    }

    override suspend fun setAccessToken(accessToken: String) {
        dataStore.edit { prefs ->
            prefs[ACCESS_TOKEN_KEY] = accessToken
        }
    }

    override suspend fun clearToken(): Result<Unit> {
        return try {
            dataStore.edit { prefs ->
                prefs.remove(ACCESS_TOKEN_KEY)
                prefs.remove(REFRESH_TOKEN_KEY)
                Timber.tag("logout").d("clearToken 성공")
            }
            Result.success(Unit)
        } catch (e: Exception) {
            Timber.tag("logout").e("clearToken 실패: ${e.message}")
            Result.failure(e)
        }
    }

    override suspend fun setRefreshToken(refreshToken: String) {
        dataStore.edit { prefs ->
            prefs[REFRESH_TOKEN_KEY] = refreshToken
        }
    }

    override suspend fun getRefreshToken(): String {
        return dataStore.data.map { prefs ->
            prefs[REFRESH_TOKEN_KEY] ?: ""
        }.first()
    }

    companion object {
        val ACCESS_TOKEN_KEY = stringPreferencesKey("access_token")
        val REFRESH_TOKEN_KEY = stringPreferencesKey("refresh_token")
    }
}