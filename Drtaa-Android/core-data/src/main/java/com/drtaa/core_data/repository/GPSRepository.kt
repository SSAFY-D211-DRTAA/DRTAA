package com.drtaa.core_data.repository

import com.drtaa.core_model.map.CarRoute
import com.drtaa.core_model.map.Destination
import com.drtaa.core_model.map.ResponseGPS
import kotlinx.coroutines.flow.Flow

interface GPSRepository {
    fun observeMqttGPSMessages(): Flow<ResponseGPS>
    fun observeMqttPathMessages(): Flow<List<CarRoute>>
    suspend fun setupMqttConnection()
    fun publishGpsData(data: String, topic: String)
    fun disconnectMqtt()
    fun observeConnectionStatus(): Flow<Int>
    fun fetchPath(data: String, topic: String)
    fun fetchDest(data: String, topic: String)
    fun observeMqttDestMessages(): Flow<Destination>
}