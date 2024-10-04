package com.d211.drtaa.domain.payment.entity;


import io.swagger.v3.oas.annotations.media.Schema;
import jakarta.persistence.*;
import lombok.*;

import java.time.LocalDateTime;

@Getter
@Setter
@Entity
@Builder
@Table(name = "payments")
@NoArgsConstructor
@AllArgsConstructor
public class Payment {

    @Id
    @Column(name = "id",nullable = false)
    @Schema(description = "유저 번호", example = "1")
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(name = "receipt_id", nullable = false)
    @Schema(description = "영수증 번호", example = "1")
    private String receiptId;

    @Column(name = "order_id", nullable = false)
    @Schema(description = "주문 번호", example = "1")
    private String orderId;

    @Column(name = "price", nullable = false)
    @Schema(description = "가격", example = "100")
    private Integer price;

    @Column(name = "payment_method", nullable = false)
    @Schema(description = "결제 방식", example = "카드")
    private String paymentMethod;

    @Column(name = "purchased_at", nullable = false)
    @Schema(description = "결제 시간", example = "date")
    private LocalDateTime purchasedAt;

    @Column(name = "user_id", nullable = false)
    @Schema(description = "유저 아이디", example = "1")
    private Long userId;

    @Column(name = "car_id", nullable = false)
    @Schema(description = "차 아이디", example = "1")
    private Long carId;

    @Column(name = "head_count", nullable = false)
    @Schema(description = "탑승 인원", example = "2")
    private Integer headCount;

    @Column(name = "rent_start_time", nullable = false)
    @Schema(description = "렌트 시작 시간", example = "date")
    private LocalDateTime rentStartTime;

    @Column(name = "rent_end_time", nullable = false)
    @Schema(description = "렌트 종료 시간", example = "date")
    private LocalDateTime rentEndTime;

}
