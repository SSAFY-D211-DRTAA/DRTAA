package com.d211.drtaa.domain.rent.dto.response;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

@Data
@AllArgsConstructor
@Builder
public class RentCarDispatchStatusResponseDTO {
    @Schema(description = "렌트 차량 고유 번호", example = "1")
    private long rentCarId;
    @Schema(description = "렌트 차량 번호", example = "123가1234")
    private String rentCarNumber;
    @Schema(description = "렌트 차량 배차 상태", example = "0")
    private boolean rentCarIsDispatch;
}
