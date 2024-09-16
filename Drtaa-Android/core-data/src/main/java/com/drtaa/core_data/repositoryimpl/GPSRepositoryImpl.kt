package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_mqtt.MqttManager
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.launch
import okhttp3.Dispatcher
import javax.inject.Inject

class GPSRepositoryImpl @Inject constructor(
    private val mqttManager: MqttManager
) {
    private val _gpsData = MutableSharedFlow<String>()
    val gpsData: Flow<String> = _gpsData

    init {
        observeMqttMessages()
    }

    private fun observeMqttMessages() {
        CoroutineScope(Dispatchers.IO).launch {
            mqttManager.receivedMessages.collect { message ->
                _gpsData.emit(message)
            }
        }
    }

    suspend fun setupMqttConnection() {
        mqttManager.setupMqttClient()
    }

    fun publishGpsData(data: String) {
        mqttManager.publishMessage(data)
    }

    fun disconnectMqtt() {
        mqttManager.disconnect()
    }
}