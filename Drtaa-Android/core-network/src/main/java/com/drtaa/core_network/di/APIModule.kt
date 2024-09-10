package com.drtaa.core_network.di

import com.drtaa.core_network.api.MapAPI
import com.drtaa.core_network.api.SignAPI
import com.drtaa.core_network.api.TestAPI
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import retrofit2.Retrofit
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object APIModule {
    @Singleton
    @Provides
    fun provideTestAPI(
        @DefaultRetrofit
        retrofit: Retrofit
    ): TestAPI = retrofit.create(TestAPI::class.java)

    @Singleton
    @Provides
    fun provideSignAPI(
        @DefaultRetrofit
        retrofit: Retrofit
    ): SignAPI = retrofit.create(SignAPI::class.java)

    @Singleton
    @Provides
    fun provideMapAPI(
        @MapRetrofit
        retrofit: Retrofit
    ): MapAPI = retrofit.create(MapAPI::class.java)
}