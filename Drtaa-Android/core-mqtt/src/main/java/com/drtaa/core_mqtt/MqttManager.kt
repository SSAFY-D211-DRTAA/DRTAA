package com.drtaa.core_mqtt

import com.drtaa.core_model.map.ResponseGPS
import com.google.gson.Gson
import com.hivemq.client.mqtt.MqttClient
import com.hivemq.client.mqtt.datatypes.MqttQos
import com.hivemq.client.mqtt.mqtt5.Mqtt5AsyncClient
import com.hivemq.client.mqtt.mqtt5.message.publish.Mqtt5Publish
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import timber.log.Timber
import javax.inject.Inject
import javax.inject.Singleton
import kotlin.math.min

private const val TAG = "MQTT"

@Singleton
class MqttManager @Inject constructor() {
    private lateinit var client: Mqtt5AsyncClient
    private val gson = Gson()
    private val _receivedMessages = MutableSharedFlow<ResponseGPS>()
    val receivedMessages: SharedFlow<ResponseGPS> = _receivedMessages.asSharedFlow()
    private var reconnectAttempts = 0
    private var isConnected = false
    private val _connectionStatus = MutableSharedFlow<Int>()
    val connectionStatus: SharedFlow<Int> = _connectionStatus.asSharedFlow()

    suspend fun setupMqttClient() {
        client = MqttClient.builder()
            .useMqttVersion5()
            .serverHost(MQTT_SERVER)
            .serverPort(PORT)
            .buildAsync()

        connectWithRetry()
    }

    private suspend fun connectWithRetry() {
        while (!isConnected) {
            try {
                withContext(Dispatchers.IO) {
                    client.connect().whenComplete { _, throwable ->
                        if (throwable != null) {
                            Timber.tag(TAG).e(throwable, "MQTT 연결실패")
                            CoroutineScope(Dispatchers.Main).launch {
                                if (reconnectAttempts == 10) {
                                    _connectionStatus.emit(0)
                                } else {
                                    _connectionStatus.emit(-1)
                                }
                            }
                        } else {
                            Timber.tag(TAG).d("MQTT 연결성공")
                            isConnected = true
                            reconnectAttempts = 0
                            subscribeToTopic()
                            CoroutineScope(Dispatchers.Main).launch {
                                _connectionStatus.emit(1)
                            }
                        }
                    }
                }
            } catch (e: Exception) {
                Timber.tag(TAG).e(e, "MQTT 연결실패")
            }

            if (!isConnected) {
                val delay = calculateReconnectDelay()
                Timber.tag(TAG).d("#$reconnectAttempts 재연결 시도 중... ${delay}ms 후 다시 시도합니다.")
                delay(delay)
            }
        }
    }

    private fun calculateReconnectDelay(): Long {
        val delay = INITIAL_RECONNECT_DELAY * (1 shl min(reconnectAttempts, DELAY))
        reconnectAttempts++
        return min(delay, MAX_RECONNECT_DELAY)
    }

    private fun subscribeToTopic() {
        client.subscribeWith()
            .topicFilter(GPS_PUB)
            .callback { publish: Mqtt5Publish ->
                val message = String(publish.payloadAsBytes)
                CoroutineScope(Dispatchers.IO).launch {
                    _connectionStatus.emit(1)
                    val parsedMessage = gson.fromJson(message, ResponseGPS::class.java)
                    _receivedMessages.emit(parsedMessage)
                    Timber.tag(TAG).d("MQTT 응답: $parsedMessage")
                }
            }
            .send()
            .whenComplete { connAck, throwable ->
                if (throwable != null) {
                    Timber.tag(TAG).e(throwable, "구독 실패")
                } else {
                    Timber.tag(TAG).d("구독 성공 $connAck")
                }
            }
    }

    fun publishMessage(message: String) {
        if (!isConnected) {
            Timber.tag(TAG).w("MQTT가 연결되어 있지 않습니다. 메시지를 보낼 수 없습니다.")
            return
        }
        client.publishWith()
            .topic(GPS_SUB)
            .qos(MqttQos.EXACTLY_ONCE)
            .payload(message.toByteArray())
            .send()
            .whenComplete { connAck, throwable ->
                if (throwable != null) {
                    Timber.tag(TAG).e(throwable, "발행 실패")
                } else {
                    Timber.tag(TAG).d("발행 성공 $connAck")
                }
            }
    }

    fun disconnect() {
        if (isConnected) {
            Timber.tag("MQTT").d("disconnect success")
            client.disconnect()
            isConnected = false
        }
    }

    companion object {
        private const val MQTT_SERVER = BuildConfig.MQTT_URL
        private const val PORT = 1883
        private const val DELAY = 5
        private const val MAX_RECONNECT_DELAY = 20000L // 최대 30초
        private const val INITIAL_RECONNECT_DELAY = 1000L // 초기 1초
        private const val GPS_SUB = "gps/data/v1/subscribe"
        private const val GPS_PUB = "gps/data/v1/publish"
    }
}