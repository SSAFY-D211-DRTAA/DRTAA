package com.drtaa.core_data.datasource

import com.drtaa.core_model.pay.PaymentCompletionInfo
import com.drtaa.core_model.network.RequestPayment
import com.drtaa.core_model.pay.ResponsePayment

interface PaymentDataSource {
    suspend fun savePaymentInfo(requestPayment: RequestPayment)
    suspend fun getPaymentInfo(receiptId: String): PaymentCompletionInfo
    suspend fun getUserPayments(): List<ResponsePayment>
}