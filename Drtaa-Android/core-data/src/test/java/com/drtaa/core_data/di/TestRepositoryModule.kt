package com.drtaa.core_data.di

import com.drtaa.core_data.datasource.NaverDataSource
import com.drtaa.core_data.repository.NaverRepository
import com.drtaa.core_data.repositoryimpl.NaverRepositoryImpl
import dagger.Module
import dagger.Provides
import dagger.hilt.components.SingletonComponent
import dagger.hilt.testing.TestInstallIn
import javax.inject.Singleton

@Module
@TestInstallIn(
    components = [SingletonComponent::class],
    replaces = [RepositoryModule::class]
)
object TestRepositoryModule {
    @Provides
    @Singleton
    fun provideNaverRepository(dataSource: NaverDataSource): NaverRepository =
        NaverRepositoryImpl(dataSource)
}