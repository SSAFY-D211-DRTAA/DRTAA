package com.drtaa.core_network.fcm

import com.google.firebase.messaging.FirebaseMessaging

class FCMTokenManager {
    fun getToken(callback: (String?) -> Unit) {
        FirebaseMessaging.getInstance().token
            .addOnCompleteListener { task ->
                if (task.isSuccessful) {
                    callback(task.result)
                } else {
                    callback(null)
                }
            }
    }

    fun sendTokenToServer(token: String) {
        // 서버에 토큰 전송 로직 구현
    }
}