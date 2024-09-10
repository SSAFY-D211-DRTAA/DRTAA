package com.drtaa.core_data.di

import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_data.repository.TokenRepository
import com.drtaa.core_data.repositoryimpl.SignRepositoryImpl
import com.drtaa.core_data.repositoryimpl.TokenRepositoryImpl
import dagger.Binds
import dagger.Module
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
interface RepositoryModule {
    @Singleton
    @Binds
    fun bindTokenRepository(tokenRepositoryImpl: TokenRepositoryImpl): TokenRepository

    @Singleton
    @Binds
    fun bindSignRepository(signRepositoryImpl: SignRepositoryImpl): SignRepository
}