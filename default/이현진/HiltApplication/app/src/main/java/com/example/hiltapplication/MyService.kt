package com.example.hiltapplication

import android.app.Service
import android.content.Intent
import android.os.IBinder
import com.example.hiltapplication.domain.repository.MyRepository
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import javax.inject.Inject

@AndroidEntryPoint
class MyService : Service() {

    @Inject
    lateinit var repository: MyRepository

    override fun onCreate() {
        super.onCreate()

        CoroutineScope(Dispatchers.IO).launch {
            repository.doNetworkCall()
        }
    }

    override fun onBind(intent: Intent?): IBinder? {
        return null
    }
}