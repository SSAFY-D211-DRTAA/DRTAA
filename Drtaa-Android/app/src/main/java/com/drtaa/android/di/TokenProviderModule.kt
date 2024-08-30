package com.drtaa.android.di

import com.drtaa.core_model.auth.TokenProvider
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object TokenProviderModule {
    @Provides
    @Singleton
    fun provideTokenProvider(tokenProviderImpl: TokenProviderImpl): TokenProvider =
        tokenProviderImpl
}