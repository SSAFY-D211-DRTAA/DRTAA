package com.drtaa.core_data.repository

import com.drtaa.core_model.pay.PaymentCompletionInfo
import com.drtaa.core_model.network.RequestPayment
import com.drtaa.core_model.pay.ResponsePayment
import kotlinx.coroutines.flow.Flow

interface PaymentRepository {
    suspend fun savePaymentInfo(requestPayment: RequestPayment): Flow<Result<Unit>>
    suspend fun getPaymentInfo(receiptId: String): Flow<Result<PaymentCompletionInfo>>
    suspend fun getUserPayments(): Flow<Result<List<ResponsePayment>>>
}