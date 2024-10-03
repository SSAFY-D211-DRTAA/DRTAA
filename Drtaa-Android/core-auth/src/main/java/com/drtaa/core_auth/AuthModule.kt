package com.drtaa.core_auth

import com.drtaa.core_data.repository.SignRepository
import com.google.android.libraries.identity.googleid.GetGoogleIdOption
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object AuthModule {
    @Provides
    @Singleton
    fun provideSocialLoginManager(
        googleIdOption: GetGoogleIdOption,
        signRepository: SignRepository,
    ): SocialLoginManager {
        return SocialLoginManager(googleIdOption, signRepository)
    }
}