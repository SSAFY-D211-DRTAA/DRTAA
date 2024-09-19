package com.drtaa.core_model.map

data class ResponseGPS(
    val header: Header,
    val latitude: Double,
    val longitude: Double,
    val altitude: Double,
    val eastOffset: Double,
    val northOffset: Double,
    val status: Int
) {
    data class Header(
        val seq: Int,
        val stamp: Stamp,
        val frameId: String
    ) {
        data class Stamp(
            val secs: Int,
            val nsecs: Int
        )
    }
}