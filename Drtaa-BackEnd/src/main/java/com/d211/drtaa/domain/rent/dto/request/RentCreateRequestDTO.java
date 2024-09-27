package com.d211.drtaa.domain.rent.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

import java.time.LocalDateTime;

@Data
@AllArgsConstructor
@Getter
public class RentCreateRequestDTO {
    @Schema(description = "렌트 이용 인원 수", example = "1(단위: 명)")
    private int rentHeadCount;
    @Schema(description = "렌트 이용 시간", example = "120(분)")
    private int rentTime;
    @Schema(description = "렌트 이용 금액", example = "0(단위: 원)")
    private long rentPrice;
    @Schema(description = "렌트 이용 시작 시간", example = "2024/01/01 00:00")
    private LocalDateTime rentStartTime;
    @Schema(description = "렌트 이용 종료 시간", example = "2024/01/01 02:00")
    private LocalDateTime rentEndTime;
    @Schema(description = "렌트 탑승 장소 위도", example = "0.0")
    private double rentDptLat;
    @Schema(description = "렌트 탑승 장소 경도", example = "0.0")
    private double rentDptLon;
    @Schema(description = "탑승 장소 이름", example = "디지털미디어시티역")
    private String datePlacesName;
    @Schema(description = "탑승 장소 카테고리", example = "지하철역")
    private String datePlacesCategory;
    @Schema(description = "탑승 장소 주소", example = "서울특별시 마포구 월드컵북로 366")
    private String datePlacesAddress;
}