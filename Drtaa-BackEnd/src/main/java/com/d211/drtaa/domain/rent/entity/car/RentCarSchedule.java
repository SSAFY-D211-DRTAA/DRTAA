package com.d211.drtaa.domain.rent.entity.car;

import com.d211.drtaa.domain.rent.entity.Rent;
import io.swagger.v3.oas.annotations.media.Schema;
import jakarta.persistence.*;
import lombok.*;

import java.time.LocalDate;

@Entity
@Table(name = "rent_car_schedule")
@NoArgsConstructor
@AllArgsConstructor
@Getter
@Setter
@Builder
public class RentCarSchedule {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "rent_car_schedule_id", nullable = false)
    @Schema(description = "렌트 차량 일정 고유 번호", example = "1")
    private long rentCarScheduleId;

    @OneToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "rent_id", nullable = false)
    @Schema(description = "렌트 고유 번호", example = "1")
    private Rent rent;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "rent_car_id", nullable = false)
    @Schema(description = "렌트한 차량 고유번호", example = "1")
    private RentCar rentCar;

    @Column(name = "rent_car_schedule_start_date", nullable = false)
    @Schema(description = "렌트 차량 일정 시작 일자", example = "2024/01/01")
    private LocalDate rentCarScheduleStartDate;

    @Column(name = "rent_car_schedule_end_date", nullable = false)
    @Schema(description = "렌트 차량 일정 종료 일자", example = "2024/01/02")
    private LocalDate rentCarScheduleEndDate;

    @Column(name = "rent_car_schedule_is_done", nullable = false)
    @Schema(description = "렌트 차량 일정 종료 여부", example = "true")
    private boolean rentCarScheduleIsDone;
}
