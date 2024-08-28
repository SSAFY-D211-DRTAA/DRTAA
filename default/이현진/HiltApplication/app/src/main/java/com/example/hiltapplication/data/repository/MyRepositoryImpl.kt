package com.example.hiltapplication.data.repository

import android.app.Application
import com.example.hiltapplication.R
import com.example.hiltapplication.data.remote.MyApi
import com.example.hiltapplication.domain.repository.MyRepository
import javax.inject.Inject

class MyRepositoryImpl @Inject constructor(
    private val api: MyApi,
    private val appContext: Application
) : MyRepository {

    init {
        val appName = appContext.getString(R.string.app_name)
        println("Hello from the repository. The app name is $appName")
    }

    override suspend fun doNetworkCall() {

    }
}