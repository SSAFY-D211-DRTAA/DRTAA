package com.d211.drtaa.domain.rent.entity.car;

import io.swagger.v3.oas.annotations.media.Schema;
import jakarta.persistence.*;
import lombok.*;
import org.hibernate.annotations.ColumnDefault;

@Entity
@Table(name = "rent_car")
@NoArgsConstructor
@AllArgsConstructor
@Getter
@Setter
@Builder
public class RentCar {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "rent_car_id", nullable = false)
    @Schema(description = "렌트 차량 고유 번호", example = "1")
    private long rentCarId;

    @Column(name = "rent_car_number", nullable = false)
    @Schema(description = "렌트 차량 번호", example = "123가1234")
    private String rentCarNumber;

    @Column(name = "rent_car_manufacturer", nullable = false)
    @Schema(description = "렌트 차량 제조사", example = "KIA")
    private String rentCarManufacturer;

    @Column(name = "rent_car_model", nullable = false)
    @Schema(description = "렌트 차량 모델 이름", example = "K5")
    private String rentCarModel;

    @Column(name = "rent_car_is_dispatch", nullable = false)
    @ColumnDefault("0")
    @Schema(description = "렌트 차량 배차 상태", example = "0")
    private boolean rentCarIsDispatch;

    @Column(name = "rent_car_driving_status", nullable = false)
    @ColumnDefault("PARKED")
    @Schema(description = "렌트 차량 주행 상태", example = "parked")
    private RentDrivingStatus rentCarDrivingStatus;
}
