package com.d211.drtaa.domain.payment.dto.response;

import lombok.Builder;
import lombok.Getter;
import lombok.Setter;

import java.time.LocalDateTime;

@Getter
@Setter
@Builder
public class PaymentResponse {
    private String receiptId;
    private String orderId;
    private Integer price;
    private String paymentMethod;
    private LocalDateTime purchasedAt;
    private Long carId;
    private Integer headCount;
    private LocalDateTime rentStartTime;
    private LocalDateTime rentEndTime;
}
