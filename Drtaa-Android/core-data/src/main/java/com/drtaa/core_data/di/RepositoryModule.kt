package com.drtaa.core_data.di

import com.drtaa.core_data.repository.GPSRepository
import com.drtaa.core_data.repository.NaverRepository
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_data.repository.TokenRepository
import com.drtaa.core_data.repository.TourRepository
import com.drtaa.core_data.repositoryimpl.GPSRepositoryImpl
import com.drtaa.core_data.repositoryimpl.NaverRepositoryImpl
import com.drtaa.core_data.repositoryimpl.SignRepositoryImpl
import com.drtaa.core_data.repositoryimpl.TokenRepositoryImpl
import com.drtaa.core_data.repositoryimpl.TourRepositoryImpl
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

    @Singleton
    @Binds
    fun bindMapRepository(mapRepositoryImpl: NaverRepositoryImpl): NaverRepository

    @Singleton
    @Binds
    fun bindTourRepository(tourRepositoryImpl: TourRepositoryImpl): TourRepository

    @Singleton
    @Binds
    fun bindGPSRepository(gpsRepositoryImpl: GPSRepositoryImpl): GPSRepository
}