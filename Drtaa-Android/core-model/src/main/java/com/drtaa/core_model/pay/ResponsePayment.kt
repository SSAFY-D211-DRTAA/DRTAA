package com.drtaa.core_model.pay

import java.time.LocalDateTime
import java.time.ZoneId
import java.util.Date

data class ResponsePayment(
    val receiptId: String,
    val orderId: String,
    val price: Int,
    val paymentMethod: String,
    val purchasedAt: String,
    val carId: Int,
    val headCount: Int,
    val rentStartTime: String,
    val rentEndTime: String
) {
    private fun String.toLocalDateTime(): LocalDateTime = LocalDateTime.parse(this)

    private fun LocalDateTime.toDate(): Date = Date.from(this.atZone(ZoneId.systemDefault()).toInstant())
}