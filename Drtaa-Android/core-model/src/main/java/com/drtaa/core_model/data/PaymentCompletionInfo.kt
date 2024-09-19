package com.drtaa.core_model.data

import com.drtaa.core_model.network.RequestPayment
import com.drtaa.core_model.util.Time.HOUR_TO_MILLIS
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

data class PaymentCompletionInfo(
    val receiptId: String,
    val orderId: String,
    val price: Int,
    val userId: String,
    val paymentMethod: String,
    val purchasedAt: Date,
    val carId: Long = 1,
    val headCount: Int = 1,
    val rentStartTime: Date = Date(),
    val rentEndTime: Date = Date(System.currentTimeMillis() + HOUR_TO_MILLIS.value) // 1시간 후
) {
    fun toPaymentRequest(): RequestPayment {
        val dateFormat = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss", Locale.getDefault())
        return RequestPayment(
            receiptId = receiptId,
            orderId = orderId,
            price = price,
            paymentMethod = paymentMethod,
            purchasedAt = dateFormat.format(purchasedAt),
            carId = carId,
            headCount = headCount,
            rentStartTime = dateFormat.format(rentStartTime),
            rentEndTime = dateFormat.format(rentEndTime)
        )
    }
}
