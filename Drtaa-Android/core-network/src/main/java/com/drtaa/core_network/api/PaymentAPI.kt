package com.drtaa.core_network.api

import com.drtaa.core_model.data.PaymentCompletionInfo
import com.drtaa.core_model.network.RequestPayment
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.POST
import retrofit2.http.Path

interface PaymentAPI {
    @POST("payment/save")
    suspend fun savePaymentInfo(@Body requsetPaymentInfo: RequestPayment)

    @GET("payment/user")
    suspend fun getUserPayments(): List<PaymentCompletionInfo>

    @GET("payment/{receiptId}")
    suspend fun getPaymentInfo(@Path("receiptId") receiptId: String): PaymentCompletionInfo
}