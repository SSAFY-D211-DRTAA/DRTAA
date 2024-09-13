package com.drtaa.core_data.di

import com.drtaa.core_data.datasource.NaverDataSource
import com.drtaa.core_data.datasource.SignDataSource
import com.drtaa.core_data.datasource.TokenDataSource
import com.drtaa.core_data.datasource.TourDataSource
import com.drtaa.core_data.datasourceimpl.NaverDataSourceImpl
import com.drtaa.core_data.datasourceimpl.SignDataSourceImpl
import com.drtaa.core_data.datasourceimpl.TokenDataSourceImpl
import com.drtaa.core_data.datasourceimpl.TourDataSourceImpl
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

    @Singleton
    @Binds
    fun bindSignDataSource(signDataSourceImpl: SignDataSourceImpl): SignDataSource

    @Singleton
    @Binds
    fun bindMapDataSource(mapDataSourceImpl: NaverDataSourceImpl): NaverDataSource

    @Singleton
    @Binds
    fun bindTourDataSource(tourDataSourceImpl: TourDataSourceImpl): TourDataSource
}