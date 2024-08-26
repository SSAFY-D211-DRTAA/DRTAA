package com.example.hiltapplication.domain.repository

interface MyRepository {
    suspend fun doNetworkCall()
}