package com.drtaa.core_mqtt.di

import com.drtaa.core_mqtt.MqttManager
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object MqttModule {
    @Provides
    @Singleton
    fun provideMqttManager(): MqttManager = MqttManager()
}