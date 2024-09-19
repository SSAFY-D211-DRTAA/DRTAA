package com.d211.drtaa.domain.payment.dto.request;

import lombok.Getter;
import lombok.Setter;

import java.time.LocalDateTime;

@Getter
@Setter
public class PaymentRequest {
    private String receiptId;
    private String orderId;
    private Integer price;
    private String paymentMethod;
    private Long carId;
    private Integer headCount;
    private LocalDateTime rentStartTime;
    private LocalDateTime rentEndTime;
}