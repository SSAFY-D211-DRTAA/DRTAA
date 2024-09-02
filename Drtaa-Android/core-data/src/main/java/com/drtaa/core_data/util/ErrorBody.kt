package com.drtaa.core_data.util

data class ErrorBody(
    val data: Data,
    val status: String
) {
    data class Data(
        val code: String,
        val error: String,
        val message: String,
        val timestamp: String
    )
}