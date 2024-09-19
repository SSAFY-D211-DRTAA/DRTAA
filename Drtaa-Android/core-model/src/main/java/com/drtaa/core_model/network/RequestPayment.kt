package com.drtaa.core_model.network

data class RequestPayment(
    val receiptId: String,
    val orderId: String,
    val price: Int,
    val paymentMethod: String,
    val purchasedAt: String,
    val carId: Long,
    val headCount: Int,
    val rentStartTime: String,
    val rentEndTime: String
)
