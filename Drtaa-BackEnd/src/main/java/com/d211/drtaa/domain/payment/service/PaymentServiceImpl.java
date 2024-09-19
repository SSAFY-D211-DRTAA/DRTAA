package com.d211.drtaa.domain.payment.service;

import com.d211.drtaa.domain.payment.dto.PaymentCompletionInfo;
import com.d211.drtaa.domain.payment.dto.request.PaymentRequest;
import com.d211.drtaa.domain.payment.dto.response.PaymentResponse;
import com.d211.drtaa.domain.payment.entity.Payment;
import com.d211.drtaa.domain.payment.repository.PaymentRepository;
import com.d211.drtaa.domain.user.repository.UserRepository;
import com.d211.drtaa.global.exception.payment.PaymentException;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.List;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
public class PaymentServiceImpl implements PaymentService {

    private final PaymentRepository paymentRepository;
    private final UserRepository userRepository;

    @Override
    @Transactional
    public PaymentResponse savePaymentInfo(PaymentRequest paymentRequest, String userProviderId) {
        Long userId = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new PaymentException.UserNotFoundException("User not found with providerId: " + userProviderId))
                .getUserId();

        Payment payment = convertToEntity(paymentRequest, userId);
        Payment savedPayment = paymentRepository.save(payment);
        return convertToResponse(savedPayment);
    }

    @Override
    @Transactional(readOnly = true)
    public List<PaymentResponse> getUserPayments(String userProviderId) {
        Long userId = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new PaymentException.UserNotFoundException("User not found with providerId: " + userProviderId))
                .getUserId();

        List<Payment> payments = paymentRepository.findByUserId(userId);
        return payments.stream()
                .map(this::convertToResponse)
                .collect(Collectors.toList());
    }

    @Override
    @Transactional(readOnly = true)
    public PaymentResponse getPaymentInfo(String receiptId, String userProviderId) {
        Long userId = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new PaymentException.UserNotFoundException("User not found with providerId: " + userProviderId))
                .getUserId();

        Payment payment = paymentRepository.findByReceiptId(receiptId)
                .orElseThrow(() -> new PaymentException.PaymentNotFoundException("Payment not found with receiptId: " + receiptId));

        if (!payment.getUserId().equals(userId)) {
            throw new PaymentException.PaymentNotFoundException("Payment not found or unauthorized");
        }

        return convertToResponse(payment);
    }

    private Payment convertToEntity(PaymentRequest dto, Long userId) {
        return Payment.builder()
                .receiptId(dto.getReceiptId())
                .orderId(dto.getOrderId())
                .price(dto.getPrice())
                .paymentMethod(dto.getPaymentMethod())
                .purchasedAt(LocalDateTime.now())
                .userId(userId)
                .carId(dto.getCarId())
                .headCount(dto.getHeadCount())
                .rentStartTime(dto.getRentStartTime())
                .rentEndTime(dto.getRentEndTime())
                .build();
    }

    private PaymentResponse convertToResponse(Payment entity) {
        return PaymentResponse.builder()
                .receiptId(entity.getReceiptId())
                .orderId(entity.getOrderId())
                .price(entity.getPrice())
                .paymentMethod(entity.getPaymentMethod())
                .purchasedAt(entity.getPurchasedAt())
                .carId(entity.getCarId())
                .headCount(entity.getHeadCount())
                .rentStartTime(entity.getRentStartTime())
                .rentEndTime(entity.getRentEndTime())
                .build();
    }
}