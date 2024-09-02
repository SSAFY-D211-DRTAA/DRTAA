package com.example.doteacher.ui.util.server

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