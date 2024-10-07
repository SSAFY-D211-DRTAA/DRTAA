package com.drtaa.feature_main

import android.app.NotificationManager
import android.app.PendingIntent
import android.content.Context
import android.content.Intent
import androidx.core.app.NotificationCompat
import com.drtaa.core_model.map.Search
import com.google.firebase.messaging.FirebaseMessagingService
import com.google.firebase.messaging.RemoteMessage
import dagger.hilt.android.AndroidEntryPoint
import timber.log.Timber

@AndroidEntryPoint
class DrtaaMessagingService : FirebaseMessagingService() {

    override fun onNewToken(token: String) {
        super.onNewToken(token)
        Timber.tag("fcm").d("$token")
    }

    override fun onMessageReceived(message: RemoteMessage) {
        super.onMessageReceived(message)
        message.data.let {
            val datePlacesName = it["datePlacesName"]
            val datePlacesCategory = it["datePlacesCategory"]
            val datePlacesAddress = it["datePlacesAddress"]
            val datePlacesLat = it["datePlacesLat"]?.toDouble()
            val datePlacesLon = it["datePlacesLon"]?.toDouble()

            val request = Search(
                title = datePlacesName.toString(),
                category = datePlacesCategory.toString(),
                roadAddress = datePlacesAddress.toString(),
                lng = datePlacesLat ?: 0.0,
                lat = datePlacesLon ?: 0.0
            )

            val mainIntent = Intent(this, MainActivity::class.java).apply {
                flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TASK
                putExtra("recommend", request)
            }

            val pIntent: PendingIntent =
                PendingIntent.getActivity(this, 0, mainIntent, PendingIntent.FLAG_IMMUTABLE)

            val builder = NotificationCompat.Builder(this, MainActivity.CHANNEL_ID)
                .setSmallIcon(R.mipmap.ic_launcher) // 반드시 작은 아이콘을 설정
                .setContentIntent(pIntent)
                .setContentTitle("여행지를 추천합니다!")
                .setContentText("현재 위치를 기준으로 추천합니다.")
                .setPriority(NotificationCompat.PRIORITY_HIGH)
                .setAutoCancel(true)

            val notificationManager: NotificationManager =
                getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager

            notificationManager.notify(MainActivity.NOTIFICATION_ID, builder.build())

            Timber.tag("fcm").d("place  recomm $request")
        }

        message.notification?.let { msg ->
            val title = msg.title
            val body = msg.body

            val mainIntent = Intent(this, MainActivity::class.java).apply {
                flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TASK
            }

            val pIntent: PendingIntent =
                PendingIntent.getActivity(this, 0, mainIntent, PendingIntent.FLAG_IMMUTABLE)

            val builder = NotificationCompat.Builder(this, MainActivity.CHANNEL_ID)
                .setSmallIcon(R.mipmap.ic_launcher)
                .setContentIntent(pIntent)
                .setContentTitle(title)
                .setContentText(body)
                .setPriority(NotificationCompat.PRIORITY_HIGH)
                .setAutoCancel(true)

            val notificationManager: NotificationManager =
                getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager

            notificationManager.notify(MainActivity.NOTIFICATION_ID, builder.build())
        }
        Timber.tag("fcm").d("onMessageReceived: ${message.notification}")
    }
}