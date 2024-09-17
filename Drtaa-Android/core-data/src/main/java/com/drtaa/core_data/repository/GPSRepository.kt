package com.drtaa.core_data.repository

import kotlinx.coroutines.flow.Flow

interface GPSRepository {
    fun observeMqttMessages(): Flow<String>
    suspend fun setupMqttConnection()
    fun publishGpsData(data: String)
    fun disconnectMqtt()
}