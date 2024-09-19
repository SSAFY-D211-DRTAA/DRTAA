package com.d211.drtaa.domain.rent.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

@Data
@AllArgsConstructor
@Getter
public class RentTimeRequestDTO {
    @Schema(description = "렌트 고유 번호", example = "1")
    private long rentId;
    @Schema(description = "렌트 이용 시간", example = "120(분)")
    private int rentTime;
}
