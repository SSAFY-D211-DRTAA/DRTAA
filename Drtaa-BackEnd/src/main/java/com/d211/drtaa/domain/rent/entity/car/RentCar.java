package com.d211.drtaa.domain.rent.entity.car;

import io.swagger.v3.oas.annotations.media.Schema;
import jakarta.persistence.*;
import lombok.*;
import org.hibernate.annotations.ColumnDefault;

import java.util.List;

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

    @Column(name = "rent_car_img", nullable = false)
    @Schema(description = "렌트 차량 예시 사진", example = "https://myd211s3bucket.s3.ap-northeast-2.amazonaws.com/profileImg/Niro.png")
    private String rentCarImg;

    @Enumerated(EnumType.STRING)
    @Column(name = "rent_car_driving_status", nullable = false)
    @ColumnDefault("parked")
    @Schema(description = "렌트 차량 주행 상태", example = "parking")
    private RentDrivingStatus rentCarDrivingStatus;

    @OneToMany(mappedBy = "rentCar", fetch = FetchType.LAZY)
    private List<RentCarSchedule> rentCarSchedule;
}
