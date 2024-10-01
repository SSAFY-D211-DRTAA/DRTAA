package com.d211.drtaa.domain.taxi.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

import java.time.LocalDateTime;

@Data
@AllArgsConstructor
@Getter
public class TaxiCreateRequestDTO {
    @Schema(description = "택시 이용 시간", example = "30(분)")
    private int taxiTime;
    @Schema(description = "택시 이용 금액", example = "10000(단위: 원)")
    private int taxiPrice;
    @Schema(description = "택시 이용 시작 시간", example = "2024/01/01 00:00")
    private LocalDateTime taxiStartTime;
    @Schema(description = "택시 이용 종료 시간", example = "2024/01/01 00:30")
    private LocalDateTime taxiEndTime;
    @Schema(description = "택시 출발 위치 위도", example = "37.5665")
    private double taxiStartLat;
    @Schema(description = "택시 출발 위치 경도", example = "126.9780")
    private double taxiStartLon;
    @Schema(description = "택시 도착 위치 위도", example = "37.5665")
    private double taxiEndLat;
    @Schema(description = "택시 도착 위치 경도", example = "126.9780")
    private double taxiEndLon;
    @Schema(description = "출발 주소", example = "서울특별시 중구 세종대로 110")
    private String startAddress;
    @Schema(description = "도착 주소", example = "서울특별시 강남구 테헤란로 521")
    private String endAddress;
}