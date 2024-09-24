package com.drtaa.core_network.di

import com.drtaa.core_network.api.MapAPI
import com.drtaa.core_network.api.PaymentAPI
import com.drtaa.core_network.api.RentAPI
import com.drtaa.core_network.api.RentCarAPI
import com.drtaa.core_network.api.SignAPI
import com.drtaa.core_network.api.TestAPI
import com.drtaa.core_network.api.TourAPI
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import retrofit2.Retrofit
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object APIModule {
    const val MAP_SEARCH_URL = "https://openapi.naver.com/v1/search/"
    const val TOUR_URL = "http://apis.data.go.kr/B551011/KorService1/"

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
        retrofitFactory: RetrofitFactory
    ): MapAPI {
        return retrofitFactory.create(MAP_SEARCH_URL)
            .create(MapAPI::class.java)
    }

    @Singleton
    @Provides
    fun provideTourAPI(
        retrofitFactory: RetrofitFactory
    ): TourAPI {
        return retrofitFactory.create(TOUR_URL)
            .create(TourAPI::class.java)
    }

    @Singleton
    @Provides
    fun providePaymentAPI(
        @AuthRetrofit
        retrofit: Retrofit
    ): PaymentAPI = retrofit.create(PaymentAPI::class.java)

    @Singleton
    @Provides
    fun provideRentAPI(
        @AuthRetrofit
        retrofit: Retrofit
    ): RentAPI {
        return retrofit.create(RentAPI::class.java)
    }

    @Singleton
    @Provides
    fun provideRentCarAPI(
        @AuthRetrofit
        retrofit: Retrofit
    ): RentCarAPI {
        return retrofit.create(RentCarAPI::class.java)
    }
}