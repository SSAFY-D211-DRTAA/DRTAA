package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.repository.GPSRepository
import com.drtaa.core_model.map.CarRoute
import com.drtaa.core_model.map.ResponseCarRoute
import com.drtaa.core_mqtt.MqttManager
import com.google.gson.Gson
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import timber.log.Timber
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