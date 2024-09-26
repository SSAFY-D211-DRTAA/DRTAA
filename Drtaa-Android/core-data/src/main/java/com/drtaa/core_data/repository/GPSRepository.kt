package com.drtaa.core_data.repository

import com.drtaa.core_model.map.CarRoute
import com.drtaa.core_model.map.ResponseGPS
import kotlinx.coroutines.flow.Flow

interface GPSRepository {
    fun observeMqttMessages(): Flow<ResponseGPS>
    suspend fun setupMqttConnection()
    fun publishGpsData(data: String)
    fun disconnectMqtt()
    fun observeConnectionStatus(): Flow<Int>
    fun getRoute(): Flow<Result<List<CarRoute>>>
}