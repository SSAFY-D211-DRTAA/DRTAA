package com.drtaa.core_network.fcm

import com.google.firebase.messaging.RemoteMessage

class FCMMessageHandler {
    fun handleMessage(remoteMessage: RemoteMessage) {
        // 메시지 처리 로직
        val notificationData = remoteMessage.data
        val title = notificationData["title"]
        val body = notificationData["body"]

        // 필요한 처리 수행 (예: 로컬 알림 표시, 데이터 저장 등)
    }
}