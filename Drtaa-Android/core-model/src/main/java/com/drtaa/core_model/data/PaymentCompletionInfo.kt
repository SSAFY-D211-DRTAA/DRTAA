package com.drtaa.core_model.data

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

    companion object {
        private val dateFormatter = DateTimeFormatter.ISO_DATE_TIME

        fun fromMap(map: Map<String, Any?>): PaymentCompletionInfo {
            return PaymentCompletionInfo(
                receiptId = map["receiptId"] as String,
                orderId = map["orderId"] as String,
                price = (map["price"] as Number).toInt(),
                userId = map["userId"] as? String ?: "",
                paymentMethod = map["paymentMethod"] as String,
                purchasedAt = (map["purchasedAt"] as? Date) ?: Date.from(LocalDateTime.parse(map["purchasedAt"] as String, dateFormatter).atZone(ZoneId.systemDefault()).toInstant()),
                carId = (map["carId"] as Number).toInt(),
                headCount = (map["headCount"] as Number).toInt(),
                rentStartTime = (map["rentStartTime"] as? Date) ?: Date.from(LocalDateTime.parse(map["rentStartTime"] as String, dateFormatter).atZone(ZoneId.systemDefault()).toInstant()),
                rentEndTime = (map["rentEndTime"] as? Date) ?: Date.from(LocalDateTime.parse(map["rentEndTime"] as String, dateFormatter).atZone(ZoneId.systemDefault()).toInstant())
            )
        }
    }
}