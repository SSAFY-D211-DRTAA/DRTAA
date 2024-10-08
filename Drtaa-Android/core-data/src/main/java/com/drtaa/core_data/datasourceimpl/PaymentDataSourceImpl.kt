package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.PaymentDataSource
import com.drtaa.core_model.pay.PaymentCompletionInfo
import com.drtaa.core_model.network.RequestPayment
import com.drtaa.core_model.pay.ResponsePayment
import com.drtaa.core_network.api.PaymentAPI
import javax.inject.Inject

class PaymentDataSourceImpl @Inject constructor(
    private val paymentAPI: PaymentAPI
) : PaymentDataSource {
    override suspend fun savePaymentInfo(requestPayment: RequestPayment) {
        paymentAPI.savePaymentInfo(requestPayment)
    }

    override suspend fun getPaymentInfo(receiptId: String): PaymentCompletionInfo {
        return paymentAPI.getPaymentInfo(receiptId)
    }

    override suspend fun getUserPayments(): List<ResponsePayment> {
        return paymentAPI.getUserPayments()
    }
}