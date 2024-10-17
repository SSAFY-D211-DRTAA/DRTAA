package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.repository.GPSRepository
import com.drtaa.core_model.map.CarRoute
import com.drtaa.core_model.map.Destination
import com.drtaa.core_mqtt.MqttManager
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import javax.inject.Inject

class GPSRepositoryImpl @Inject constructor(
    private val mqttManager: MqttManager,
) : GPSRepository {

    override fun observeMqttGPSMessages() = flow {
        mqttManager.receivedGPSMessages.collect { message ->
            emit(message)
        }
    }

    override fun observeMqttPathMessages(): Flow<List<CarRoute>> = flow {
        mqttManager.receivedPathMessages.collect { message ->
            emit(message.msg.path)
        }
    }

    override fun observeMqttDestMessages(): Flow<Destination> = flow {
        mqttManager.receivedDestMessages.collect { message ->
            message.msg.name?.let {
                emit(message.msg)
            }
        }
    }

    override fun observeConnectionStatus() = flow {
        mqttManager.connectionStatus.collect { status ->
            emit(status)
        }
    }

    override fun fetchDest(data: String, topic: String) {
        mqttManager.publishMessage(data, topic)
    }

    override fun fetchPath(data: String, topic: String) {
        mqttManager.publishMessage(data, topic)
    }

    override suspend fun setupMqttConnection() {
        mqttManager.setupMqttClient()
    }

    override fun publishGpsData(data: String, topic: String) {
        mqttManager.publishMessage(data, topic)
    }

    override fun disconnectMqtt() {
        mqttManager.disconnect()
    }
}