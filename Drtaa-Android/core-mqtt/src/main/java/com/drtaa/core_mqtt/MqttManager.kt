package com.drtaa.core_mqtt

import com.hivemq.client.mqtt.MqttClient
import com.hivemq.client.mqtt.mqtt5.Mqtt5AsyncClient
import com.hivemq.client.mqtt.mqtt5.message.publish.Mqtt5Publish
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import timber.log.Timber
import javax.inject.Inject
import javax.inject.Singleton

private const val TAG = "MQTT"

@Singleton
class MqttManager @Inject constructor() {
    private lateinit var client: Mqtt5AsyncClient

    suspend fun setupMqttClient() {
        client = MqttClient.builder()
            .useMqttVersion5()
            .serverHost(MQTT_SERVER)
            .serverPort(PORT)
            .buildAsync()

        try {
            withContext(Dispatchers.IO) {
                client.connect().whenComplete { _, throwable ->
                    if (throwable != null) {
                        Timber.tag(TAG).e("MQTT 연결실패", throwable)
                    } else {
                        Timber.tag(TAG).d("MQTT 연결성공")
                        subscribeToTopic()
                    }
                }
            }
        } catch (e: Exception) {
            Timber.tag(TAG).e("MQTT 연결실패", e)
        }
    }

    private fun subscribeToTopic() {
        client.subscribeWith()
            .topicFilter(GPS_PUB)
            .callback { publish: Mqtt5Publish ->
                val message = String(publish.payloadAsBytes)
                Timber.tag(TAG).d("MQTT 응답: $message")
            }
            .send()
            .whenComplete { connAck, throwable ->
                if (throwable != null) {
                    Timber.tag(TAG).e("구독 실패", throwable)
                } else {
                    Timber.tag(TAG).d("구독 성공 $connAck")
                }
            }
    }

    fun publishMessage(message: String) {
        client.publishWith()
            .topic(GPS_SUB)
            .payload(message.toByteArray())
            .send()
            .whenComplete { connAck, throwable ->
                if (throwable != null) {
                    Timber.tag(TAG).e("발행 실패", throwable)
                } else {
                    Timber.tag(TAG).d("발행 성공 $connAck")
                }
            }
    }

    fun disconnect() {
        client.disconnect()
    }

    companion object {
        private const val MQTT_SERVER = "192.168.100.199"
        private const val PORT = 1883
        private const val GPS_SUB = "gps/data/v1/subscribe"
        private const val GPS_PUB = "gps/data/v1/publish"
//        private const val GPS_CMD_SUB = "cmd/data/v1/subscribe"
//        private const val GPS_CMD_PUB = "cmd/data/v1/publish"
    }
}