package com.drtaa.core_data.di

import com.drtaa.core_data.datasource.TokenDataSource
import com.drtaa.core_data.datasourceimpl.TokenDataSourceImpl
import dagger.Binds
import dagger.Module
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
interface DataSourceModule {
    @Singleton
    @Binds
    fun bindTokenDataSource(tokenDataSourceImpl: TokenDataSourceImpl): TokenDataSource
}