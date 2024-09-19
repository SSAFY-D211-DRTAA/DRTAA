package com.d211.drtaa.domain.rent.entity;

import com.d211.drtaa.domain.rent.entity.car.RentCar;
import com.d211.drtaa.domain.travel.entity.Travel;
import com.d211.drtaa.domain.user.entity.User;
import io.swagger.v3.oas.annotations.media.Schema;
import jakarta.persistence.*;
import lombok.*;
import org.hibernate.annotations.ColumnDefault;

import java.time.LocalDateTime;

@Entity
@Table(name = "rent")
@NoArgsConstructor
@AllArgsConstructor
@Getter
@Setter
@Builder
public class Rent {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "rent_id", nullable = false)
    @Schema(description = "렌트 고유 번호", example = "1")
    private long rentId;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "user_id", nullable = false)
    @Schema(description = "렌트한 회원 고유번호", example = "1")
    private User user;

    @OneToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "rent_car_id", nullable = false)
    @Schema(description = "렌트한 차량 고유번호", example = "1")
    private RentCar rentCar;

    @OneToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "travel_id", nullable = false)
    @Schema(description = "렌트한 여행 고유번호", example = "1")
    private Travel travel;

    @Column(name = "rent_status", nullable = false)
    @ColumnDefault("RESERVED")
    @Enumerated(EnumType.STRING)
    @Schema(description = "렌트 상태", example = "RESERVED")
    private RentStatus rentStatus;

    @Column(name = "rent_head_count", nullable = false)
    @ColumnDefault("1")
    @Schema(description = "렌트 이용 인원 수", example = "1(단위: 명)")
    private int rentHeadCount;

    @Column(name = "rent_price", nullable = false)
    @ColumnDefault("0")
    @Schema(description = "렌트 이용 금액", example = "0(단위: 원)")
    private long rentPrice;

    @Column(name = "rent_time", nullable = false)
    @ColumnDefault("120")
    @Schema(description = "렌트 이용 시간", example = "120(분)")
    private int rentTime;

    @Column(name = "rent_start_time", nullable = false)
    @Schema(description = "렌트 이용 시작 시간", example = "2024/01/01 00:00")
    private LocalDateTime rentStartTime;

    @Column(name = "rent_end_time", nullable = false)
    @Schema(description = "렌트 이용 종료 시간", example = "2024/01/01 02:00")
    private LocalDateTime rentEndTime;

    @Column(name = "rent_dpt_lat", nullable = false)
    @ColumnDefault("0.0")
    @Schema(description = "렌트 탑승 장소 위도", example = "0.0")
    private double rentDptLat;

    @Column(name = "rent_dpt_lon", nullable = false)
    @ColumnDefault("0.0")
    @Schema(description = "렌트 탑승 장소 경도", example = "0.0")
    private double rentDptLon;

    @Column(name = "rent_created_at", nullable = false)
    @Schema(description = "렌트 예약 생성 일시", example = "2024/01/01 00:00")
    private LocalDateTime rentCreatedAt;

    @PrePersist
    protected void onCreate() {
        this.rentCreatedAt = LocalDateTime.now();
    }
}
