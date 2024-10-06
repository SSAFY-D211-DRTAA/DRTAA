package com.drtaa.core_model.pay

import com.drtaa.core_model.network.RequestPayment
import com.drtaa.core_model.util.Time.HOUR_TO_MILLIS
import java.time.LocalDateTime
import java.time.ZoneId
import java.time.format.DateTimeFormatter
import java.util.Date

data class PaymentCompletionInfo(
    val receiptId: String,
    val orderId: String,
    val price: Int,
    val userId: String,
    val paymentMethod: String,
    val purchasedAt: Date,
    val carId: Int = 1,
    val headCount: Int = 1,
    val rentStartTime: Date = Date(),
    val rentEndTime: Date = Date(System.currentTimeMillis() + HOUR_TO_MILLIS.value)
) {
    // LocalDateTime으로 변환하는 확장 프로퍼티
    private val Date.toLocalDateTime: LocalDateTime
        get() = LocalDateTime.ofInstant(this.toInstant(), ZoneId.systemDefault())

    fun toPaymentRequest(): RequestPayment {
        val dateFormatter = DateTimeFormatter.ISO_DATE_TIME
        return RequestPayment(
            receiptId = receiptId,
            orderId = orderId,
            price = price,
            paymentMethod = paymentMethod,
            purchasedAt = purchasedAt.toLocalDateTime.format(dateFormatter),
            carId = carId,
            headCount = headCount,
            rentStartTime = rentStartTime.toLocalDateTime.format(dateFormatter),
            rentEndTime = rentEndTime.toLocalDateTime.format(dateFormatter)
        )
    }
}