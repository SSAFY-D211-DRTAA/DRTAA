package com.drtaa.core_network.di

import com.drtaa.core_network.BuildConfig
import com.drtaa.core_network.util.isJsonArray
import com.drtaa.core_network.util.isJsonObject
import com.google.gson.Gson
import com.google.gson.GsonBuilder
import com.google.gson.JsonParser
import com.google.gson.JsonSyntaxException
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import okhttp3.OkHttpClient
import okhttp3.logging.HttpLoggingInterceptor
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory
import retrofit2.converter.scalars.ScalarsConverterFactory
import timber.log.Timber
import java.util.concurrent.TimeUnit
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object NetworkModule {
    @Provides
    @Singleton
    fun provideGson(): Gson = GsonBuilder().setLenient().create()

    @Provides
    @Singleton
    fun provideRetrofitFactory(
        @DefaultOkHttpClient okHttpClient: OkHttpClient,
        gson: Gson
    ): RetrofitFactory {
        return RetrofitFactory(okHttpClient, gson)
    }

    @Singleton
    @DefaultRetrofit
    @Provides
    fun provideRetrofit(
        @DefaultOkHttpClient okHttpClient: OkHttpClient,
        gson: Gson
    ): Retrofit {
        return Retrofit.Builder()
            .addConverterFactory(ScalarsConverterFactory.create())
            .addConverterFactory(GsonConverterFactory.create(gson))
            .baseUrl(BuildConfig.BASE_URL)
            .client(okHttpClient)
            .build()
    }

    @Singleton
    @AuthRetrofit
    @Provides
    fun provideAuthRetrofit(
        @AuthOkHttpClient okHttpClient: OkHttpClient,
        gson: Gson
    ): Retrofit {
        return Retrofit.Builder()
            .addConverterFactory(ScalarsConverterFactory.create())
            .addConverterFactory(GsonConverterFactory.create(gson))
            .baseUrl(BuildConfig.BASE_URL)
            .client(okHttpClient)
            .build()
    }

    @Singleton
    @DefaultOkHttpClient
    @Provides
    fun provideOkHttpClient(
        logger: HttpLoggingInterceptor,
    ) = OkHttpClient.Builder().run {
        addInterceptor(logger)
        connectTimeout(NETWORK_TIMEOUT, TimeUnit.SECONDS)
        readTimeout(NETWORK_TIMEOUT, TimeUnit.SECONDS)
        writeTimeout(NETWORK_TIMEOUT, TimeUnit.SECONDS)
        build()
    }

    @Singleton
    @AuthOkHttpClient
    @Provides
    fun provideAuthOkHttpClient(
        logger: HttpLoggingInterceptor,
        interceptor: AccessTokenInterceptor
    ) = OkHttpClient.Builder().run {
        addInterceptor(logger)
        addInterceptor(interceptor)
        connectTimeout(NETWORK_TIMEOUT, TimeUnit.SECONDS)
        readTimeout(NETWORK_TIMEOUT, TimeUnit.SECONDS)
        writeTimeout(NETWORK_TIMEOUT, TimeUnit.SECONDS)
        build()
    }

    @Singleton
    @Provides
    fun provideLoggingInterceptor(): HttpLoggingInterceptor {
        val loggingInterceptor = HttpLoggingInterceptor {
            when {
                !it.isJsonArray() && !it.isJsonObject() ->
                    Timber.tag("RETROFIT").d("CONNECTION INFO: $it")

                else -> try {
                    Timber.tag("RETROFIT").d(
                        GsonBuilder().setPrettyPrinting().create().toJson(
                            JsonParser().parse(it)
                        )
                    )
                } catch (m: JsonSyntaxException) {
                    Timber.tag("RETROFIT").d(it)
                }
            }
        }
        loggingInterceptor.level = HttpLoggingInterceptor.Level.BODY
        return loggingInterceptor
    }

    const val NETWORK_TIMEOUT = 10L
}