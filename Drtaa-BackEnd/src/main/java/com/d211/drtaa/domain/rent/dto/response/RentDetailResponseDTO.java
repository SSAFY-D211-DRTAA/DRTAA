package com.d211.drtaa.domain.rent.dto.response;

import com.d211.drtaa.domain.rent.entity.RentStatus;
import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

import java.time.LocalDateTime;

@Data
@AllArgsConstructor
@Builder
public class RentDetailResponseDTO {
    // rent
    @Schema(description = "렌트 상태", example = "RESERVED")
    private RentStatus rentStatus;
    @Schema(description = "렌트 이용 인원 수", example = "1(단위: 명)")
    private int rentHeadCount;
    @Schema(description = "렌트 이용 금액", example = "0(단위: 원)")
    private long rentPrice;
    @Schema(description = "렌트 이용 시간", example = "120(분)")
    private int rentTime;
    @Schema(description = "렌트 이용 시작 시간", example = "2024/01/01 00:00")
    private LocalDateTime rentStartTime;
    @Schema(description = "렌트 이용 종료 시간", example = "2024/01/01 02:00")
    private LocalDateTime rentEndTime;
    @Schema(description = "렌트 탑승 장소 위도", example = "0.0")
    private double rentDptLat;
    @Schema(description = "렌트 탑승 장소 경도", example = "0.0")
    private double rentDptLon;
    @Schema(description = "렌트 예약 생성 일시", example = "2024/01/01 00:00")
    private LocalDateTime rentCreatedAt;

    // rent-car
    @Schema(description = "렌트 차량 고유 번호", example = "1")
    private long rentCarId;
    @Schema(description = "렌트 차량 번호", example = "123가1234")
    private String rentCarNumber;
    @Schema(description = "렌트 차량 제조사", example = "KIA")
    private String rentCarManufacturer;
    @Schema(description = "렌트 차량 모델 이름", example = "K5")
    private String rentCarModel;
    @Schema(description = "렌트 차량 일정 고유 번호", example = "1")
    private long rentCarScheduleId;
}
