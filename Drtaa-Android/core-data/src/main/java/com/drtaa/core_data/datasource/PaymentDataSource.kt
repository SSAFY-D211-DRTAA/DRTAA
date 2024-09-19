package com.drtaa.core_data.datasource

import com.drtaa.core_model.data.PaymentCompletionInfo
import com.drtaa.core_model.network.RequestPayment

interface PaymentDataSource {
    suspend fun savePaymentInfo(requestPayment: RequestPayment)
    suspend fun getPaymentInfo(receiptId: String): PaymentCompletionInfo
    suspend fun getUserPayments(): List<PaymentCompletionInfo>
}