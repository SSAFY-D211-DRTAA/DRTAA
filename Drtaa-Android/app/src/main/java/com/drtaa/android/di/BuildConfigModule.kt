package com.drtaa.android.di

import com.drtaa.android.BuildConfig
import com.google.android.libraries.identity.googleid.GetGoogleIdOption
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object BuildConfigModule {

    @Singleton
    @Provides
    fun provideGoogleIdOption(): GetGoogleIdOption {
        return GetGoogleIdOption.Builder()
            .setFilterByAuthorizedAccounts(false)
            .setServerClientId(BuildConfig.GOOGLE_LOGIN_CLIENT_ID)
            .setAutoSelectEnabled(false)
            .build()
    }

}