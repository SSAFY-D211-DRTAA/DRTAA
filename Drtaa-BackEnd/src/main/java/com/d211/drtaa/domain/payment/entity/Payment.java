package com.d211.drtaa.domain.payment.entity;


import io.swagger.v3.oas.annotations.media.Schema;
import jakarta.persistence.*;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;

import java.time.LocalDateTime;

@Getter
@Setter
@Entity
@Builder
@Table(name = "payments")
public class Payment {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(nullable = false)
    @Schema(description = "영수증 번호", example = "1")
    private String receiptId;

    @Column(nullable = false)
    @Schema(description = "주문 번호", example = "1")
    private String orderId;

    @Column(nullable = false)
    @Schema(description = "가격", example = "100")
    private Integer price;

    @Column(nullable = false)
    @Schema(description = "결제 방식", example = "카드")
    private String paymentMethod;

    @Column(nullable = false)
    @Schema(description = "결제 시간", example = "date")
    private LocalDateTime purchasedAt;

    @Column(nullable = false)
    @Schema(description = "유저 아이디", example = "1")
    private Long userId;

    @Column(nullable = false)
    @Schema(description = "차 아이디", example = "1")
    private Long carId;

    @Column(nullable = false)
    @Schema(description = "탑승 인원", example = "2")
    private Integer headCount;

    @Column(nullable = false)
    @Schema(description = "렌트 시작 시간", example = "date")
    private LocalDateTime rentStartTime;

    @Column(nullable = false)
    @Schema(description = "렌트 종료 시간", example = "date")
    private LocalDateTime rentEndTime;

}
