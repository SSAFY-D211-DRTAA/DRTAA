package com.drtaa.core_data.datasourceimpl

import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.stringPreferencesKey
import com.drtaa.core_data.datasource.TokenDataSource
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.map
import javax.inject.Inject
import javax.inject.Named

class TokenDataSourceImpl @Inject constructor(
    @Named("TOKEN_DATASTORE")
    private val dataStore: DataStore<Preferences>
) : TokenDataSource {
    override suspend fun getAccessToken(): String {
        return dataStore.data.map { prefs ->
            prefs[ACCESS_TOKEN_KEY] ?: ""
        }.first()
    }

    override suspend fun setAccessToken(accessToken: String) {
        dataStore.edit { prefs ->
            prefs[ACCESS_TOKEN_KEY] = accessToken
        }
    }

    override suspend fun clearToken() {
        dataStore.edit { prefs ->
            prefs.remove(ACCESS_TOKEN_KEY)
            prefs.remove(REFRESH_TOKEN_KEY)
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