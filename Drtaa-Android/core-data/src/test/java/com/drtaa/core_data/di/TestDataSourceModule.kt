package com.drtaa.core_data.di

import com.drtaa.core_data.datasource.NaverDataSource
import com.drtaa.core_data.datasource.SignDataSource
import dagger.Module
import dagger.Provides
import dagger.hilt.components.SingletonComponent
import dagger.hilt.testing.TestInstallIn
import io.mockk.mockk
import javax.inject.Singleton

@Module
@TestInstallIn(
    components = [SingletonComponent::class],
    replaces = [DataSourceModule::class]
)
object TestDataSourceModule {
    @Provides
    @Singleton
    fun provideNaverDataSource(): NaverDataSource = mockk()

    @Provides
    @Singleton
    fun provideSignDataSource(): SignDataSource = mockk()
}