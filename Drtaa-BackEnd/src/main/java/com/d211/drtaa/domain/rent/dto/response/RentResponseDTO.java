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
public class RentResponseDTO {
    @Schema(description = "렌트 고유 번호", example = "1")
    private long rentId;
    @Schema(description = "렌트 상태", example = "RESERVED")
    private RentStatus rentStatus;
    @Schema(description = "렌트 이용 인원 수", example = "1(단위: 명)")
    private int rentHeadCount;
    @Schema(description = "렌트 이용 시간", example = "120(분)")
    private int rentTime;
    @Schema(description = "렌트 이용 시작 시간", example = "2024/01/01 00:00")
    private LocalDateTime rentStartTime;
    @Schema(description = "여행 이름", example = "서울 여행")
    private String travelName;
}
