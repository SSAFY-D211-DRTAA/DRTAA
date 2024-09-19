package com.d211.drtaa.domain.payment.controller;

import com.d211.drtaa.domain.payment.dto.response.PaymentResponse;
import com.d211.drtaa.domain.payment.dto.request.PaymentRequest;
import com.d211.drtaa.domain.payment.service.PaymentService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.Authentication;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequestMapping("/payment")
@RequiredArgsConstructor
@Tag(name = "Payment", description = "결제 관련 API")
public class PaymentController {

    private final PaymentService paymentService;

    @PostMapping("/save")
    @Operation(summary = "결제 정보 저장", description = "결제 완료 정보를 저장합니다.")
    public ResponseEntity<PaymentResponse> savePaymentInfo(@RequestBody PaymentRequest paymentRequest,
                                                           Authentication authentication) {
        String userProviderId = authentication.getName();
        PaymentResponse savedPayment = paymentService.savePaymentInfo(paymentRequest, userProviderId);
        return ResponseEntity.ok(savedPayment);
    }

    @GetMapping("/user")
    @Operation(summary = "사용자 결제 내역 조회", description = "현재 로그인한 사용자의 모든 결제 내역을 조회합니다.")
    public ResponseEntity<List<PaymentResponse>> getUserPayments(Authentication authentication) {
        String userProviderId = authentication.getName();
        List<PaymentResponse> payments = paymentService.getUserPayments(userProviderId);
        return ResponseEntity.ok(payments);
    }

    @GetMapping("/{receiptId}")
    @Operation(summary = "특정 결제 정보 조회", description = "특정 영수증 ID에 해당하는 결제 정보를 조회합니다.")
    public ResponseEntity<PaymentResponse> getPaymentInfo(@PathVariable String receiptId,
                                                          Authentication authentication) {
        String userProviderId = authentication.getName();
        PaymentResponse paymentInfo = paymentService.getPaymentInfo(receiptId, userProviderId);
        return ResponseEntity.ok(paymentInfo);
    }
}