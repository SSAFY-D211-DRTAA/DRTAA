package com.drtaa.core_network.di

import com.drtaa.core_network.util.isJsonArray
import com.drtaa.core_network.util.isJsonObject
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
//    const val BASE_URL = "http://192.168.0.12:8080/" // 나중에 local.properties로 뺼 예정
    const val BASE_URL = "http://192.168.100.185:8080/" // 나중에 local.properties로 뺼 예정

    @Singleton
    @DefaultRetrofit
    @Provides
    fun provideRetrofit(@DefaultOkHttpClient okHttpClient: OkHttpClient): Retrofit {
        return Retrofit.Builder()
            .addConverterFactory(ScalarsConverterFactory.create())
            .addConverterFactory(GsonConverterFactory.create(GsonBuilder().setLenient().create()))
            .baseUrl(BASE_URL)
            .client(okHttpClient)
            .build()
    }

    @Singleton
    @AuthRetrofit
    @Provides
    fun provideAuthRetrofit(@AuthOkHttpClient okHttpClient: OkHttpClient): Retrofit {
        return Retrofit.Builder()
            .addConverterFactory(ScalarsConverterFactory.create())
            .addConverterFactory(GsonConverterFactory.create(GsonBuilder().setLenient().create()))
            .baseUrl(BASE_URL)
            .client(okHttpClient)
            .build()
    }

    @Singleton
    @DefaultOkHttpClient
    @Provides
    fun provideOkHttpClient(logger: HttpLoggingInterceptor) = OkHttpClient.Builder().run {
        connectTimeout(120, TimeUnit.SECONDS)
        readTimeout(120, TimeUnit.SECONDS)
        writeTimeout(120, TimeUnit.SECONDS)
        addInterceptor(logger)
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
        connectTimeout(120, TimeUnit.SECONDS)
        readTimeout(120, TimeUnit.SECONDS)
        writeTimeout(120, TimeUnit.SECONDS)
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
}