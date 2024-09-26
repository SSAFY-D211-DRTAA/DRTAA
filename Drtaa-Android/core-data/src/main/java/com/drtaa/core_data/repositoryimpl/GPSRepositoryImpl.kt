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
//        mqttManager.receivedPathMessages.collect { message ->
//            emit(message.msg.path)
//        }
        val dummyRouteJson =
            "{\"msg\": {\"path\": [{\"index\": 0, \"lat\": 37.57528385687211, \"lon\": 126.9010292681235}, {\"index\": 1, \"lat\": 37.57540957254328, \"lon\": 126.90083280209342}, {\"index\": 2, \"lat\": 37.57577956314894, \"lon\": 126.90020876936028}, {\"index\": 3, \"lat\": 37.57582489417089, \"lon\": 126.90008999765104}, {\"index\": 4, \"lat\": 37.576174461953926, \"lon\": 126.89957607504506}, {\"index\": 5, \"lat\": 37.57618929055784, \"lon\": 126.8995354272828}, {\"index\": 6, \"lat\": 37.57619255411515, \"lon\": 126.89947340450385}, {\"index\": 7, \"lat\": 37.57617534055685, \"lon\": 126.89940977534089}, {\"index\": 8, \"lat\": 37.575652117936585, \"lon\": 126.89879524265199}, {\"index\": 9, \"lat\": 37.575626384064876, \"lon\": 126.89874153350581}, {\"index\": 10, \"lat\": 37.57561959872729, \"lon\": 126.89870873683381}, {\"index\": 11, \"lat\": 37.575628050255006, \"lon\": 126.89864931040758}, {\"index\": 12, \"lat\": 37.576000397800286, \"lon\": 126.89809161748852}, {\"index\": 13, \"lat\": 37.576241738015, \"lon\": 126.8978396317452}, {\"index\": 14, \"lat\": 37.57651014253264, \"lon\": 126.89760349762203}, {\"index\": 15, \"lat\": 37.57663864060148, \"lon\": 126.89746909378707}, {\"index\": 16, \"lat\": 37.576834975378745, \"lon\": 126.8973102738093}, {\"index\": 17, \"lat\": 37.576864628210586, \"lon\": 126.89729728132967}, {\"index\": 18, \"lat\": 37.576900450117606, \"lon\": 126.89729532327355}, {\"index\": 19, \"lat\": 37.576962174560585, \"lon\": 126.89733024789062}, {\"index\": 20, \"lat\": 37.577261843769314, \"lon\": 126.89765637376608}, {\"index\": 21, \"lat\": 37.57728943768548, \"lon\": 126.89772050701723}, {\"index\": 22, \"lat\": 37.57729033800001, \"lon\": 126.89779913469008}, {\"index\": 23, \"lat\": 37.57725244776715, \"lon\": 126.89787799154934}, {\"index\": 24, \"lat\": 37.57686376931795, \"lon\": 126.8985133767885}, {\"index\": 25, \"lat\": 37.57665790799037, \"lon\": 126.8988207721762}]}}"
        val dummyRoute = Gson().fromJson(dummyRouteJson, ResponseCarRoute::class.java)
        Timber.tag("path").d("$dummyRoute")
        emit(dummyRoute.msg.path)
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