package com.d211.drtaa.domain.rent.dto.response;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

import java.time.LocalDateTime;

@Data
@AllArgsConstructor
@Builder
public class UserHistoryResponseDTO {
    @Schema(description = "렌트 기록 고유 번호", example = "1")
    private long renHistoryId;
    @Schema(description = "렌트 이용 시작 시간", example = "2024/01/01 00:00")
    private LocalDateTime rentStartTime;
    @Schema(description = "렌트 이용 금액", example = "0(단위: 원)")
    private long rentPrice;
}
