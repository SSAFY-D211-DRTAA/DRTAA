package com.d211.drtaa.domain.taxi.dto.response;

import com.d211.drtaa.domain.rent.entity.RentStatus;
import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

import java.time.LocalDateTime;

@Data
@AllArgsConstructor
@Builder
public class TaxiDetailResponseDTO {
    @Schema(description = "택시 고유 번호", example = "1")
    private long taxiId;
    @Schema(description = "택시 상태", example = "RESERVED")
    private RentStatus taxiStatus;
    @Schema(description = "택시 이용 금액", example = "10000(단위: 원)")
    private int taxiPrice;
    @Schema(description = "택시 이용 시간", example = "30(분)")
    private int taxiTime;
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
    @Schema(description = "택시 예약 생성 일시", example = "2024/01/01 00:00")
    private LocalDateTime taxiCreatedAt;

    @Schema(description = "택시 차량 고유 번호", example = "1")
    private long taxiCarId;
    @Schema(description = "택시 차량 번호", example = "서울 12가 3456")
    private String taxiCarNumber;
    @Schema(description = "택시 차량 제조사", example = "현대")
    private String taxiCarManufacturer;
    @Schema(description = "택시 차량 모델 이름", example = "소나타")
    private String taxiCarModel;
    @Schema(description = "택시 차량 예시 사진", example = "https://myd211s3bucket.s3.ap-northeast-2.amazonaws.com/profileImg/Niro.png")
    private String taxiCarImg;
    @Schema(description = "택시 차량 일정 고유 번호", example = "1")
    private int taxiCarScheduleId;

    @Schema(description = "출발 주소", example = "서울특별시 중구 세종대로 110")
    private String startAddress;
    @Schema(description = "도착 주소", example = "서울특별시 강남구 테헤란로 521")
    private String endAddress;
}