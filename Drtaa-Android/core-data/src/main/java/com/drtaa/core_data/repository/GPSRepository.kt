package com.drtaa.core_data.repository

import com.drtaa.core_model.map.CarRoute
import com.drtaa.core_model.map.ResponseGPS
import kotlinx.coroutines.flow.Flow

interface GPSRepository {
    fun observeMqttGPSMessages(): Flow<ResponseGPS>
    fun observeMqttPathMessages(): Flow<List<CarRoute>>
    suspend fun setupMqttConnection()
    fun publishGpsData(data: String)
    fun disconnectMqtt()
    fun observeConnectionStatus(): Flow<Int>
}