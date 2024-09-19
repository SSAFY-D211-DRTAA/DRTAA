package com.d211.drtaa.domain.payment.service;

import com.d211.drtaa.domain.payment.dto.request.PaymentRequest;
import com.d211.drtaa.domain.payment.dto.response.PaymentResponse;

import java.util.List;

public interface PaymentService {

    PaymentResponse savePaymentInfo(PaymentRequest paymentRequest, String userProviderId);

    List<PaymentResponse> getUserPayments(String userProviderId);

    PaymentResponse getPaymentInfo(String receiptId, String userProviderId);
}