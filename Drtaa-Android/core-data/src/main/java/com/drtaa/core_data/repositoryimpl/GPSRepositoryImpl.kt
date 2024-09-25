package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.repository.GPSRepository
import com.drtaa.core_mqtt.MqttManager
import kotlinx.coroutines.flow.flow
import javax.inject.Inject

class GPSRepositoryImpl @Inject constructor(
    private val mqttManager: MqttManager
) : GPSRepository {

    override fun observeMqttMessages() = flow {
        mqttManager.receivedMessages.collect { message ->
            emit(message)
        }
    }

    override fun observeConnectionStatus() = flow {
        mqttManager.connectionStatus.collect { status ->
            emit(status)
        }
    }

    override suspend fun setupMqttConnection() {
        mqttManager.setupMqttClient()
    }

    override fun publishGpsData(data: String) {
        mqttManager.publishMessage(data)
    }

    override fun disconnectMqtt() {
        mqttManager.disconnect()
    }
}