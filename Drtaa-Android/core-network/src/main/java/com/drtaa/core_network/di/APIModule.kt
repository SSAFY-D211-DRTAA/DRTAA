package com.drtaa.core_network.di

import com.drtaa.core_network.api.GeoAPI
import com.drtaa.core_network.api.MapAPI
import com.drtaa.core_network.api.PaymentAPI
import com.drtaa.core_network.api.PlanAPI
import com.drtaa.core_network.api.RentAPI
import com.drtaa.core_network.api.RentCarAPI
import com.drtaa.core_network.api.SignAPI
import com.drtaa.core_network.api.TaxiAPI
import com.drtaa.core_network.api.TmapAPI
import com.drtaa.core_network.api.PlanAPI
import com.drtaa.core_network.api.TourAPI
import com.drtaa.core_network.api.TravelAPI
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
    const val MAP_GEOCODE_URL = "https://naveropenapi.apigw.ntruss.com/"
    const val TOUR_URL = "http://apis.data.go.kr/B551011/KorService1/"
    const val TMAP_URL = "https://apis.openapi.sk.com/"

    @Singleton
    @NoAuth
    @Provides
    fun provideSignAPINoAuth(
        @DefaultRetrofit
        retrofit: Retrofit
    ): SignAPI = retrofit.create(SignAPI::class.java)

    @Singleton
    @Auth
    @Provides
    fun provideSignAPI(
        @AuthRetrofit
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
    fun provideGeoAPI(
        retrofitFactory: RetrofitFactory
    ): GeoAPI {
        return retrofitFactory.create(MAP_GEOCODE_URL)
            .create(GeoAPI::class.java)
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

    @Singleton
    @Provides
    fun providePlanAPI(
        @AuthRetrofit
        retrofit: Retrofit
    ): PlanAPI = retrofit.create(PlanAPI::class.java)

    @Singleton
    @Provides
    fun provideTaxiAPI(
        @AuthRetrofit
        retrofit: Retrofit
    ): TaxiAPI {
        return retrofit.create(TaxiAPI::class.java)
    }

    @Singleton
    @Provides
    fun provideTmapAPI(
        retrofitFactory: RetrofitFactory
    ): TmapAPI {
        return retrofitFactory.create(TMAP_URL)
            .create(TmapAPI::class.java)
    }

    @Singleton
    @Auth
    @Provides
    fun provideTravelAuthAPI(
        @AuthRetrofit
        retrofit: Retrofit
    ): TravelAPI = retrofit.create(TravelAPI::class.java)

    @Singleton
    @NoAuth
    @Provides
    fun provideTravelNoAuthAPI(
        retrofitFactory: RetrofitFactory
    ): TravelAPI {
        return retrofitFactory.create(MAP_SEARCH_URL)
            .create(TravelAPI::class.java)
    }
}