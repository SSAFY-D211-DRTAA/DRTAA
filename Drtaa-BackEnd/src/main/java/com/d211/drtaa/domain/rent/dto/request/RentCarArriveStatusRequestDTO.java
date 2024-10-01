package com.d211.drtaa.domain.rent.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

@Data
@AllArgsConstructor
@Getter
public class RentCarArriveStatusRequestDTO {
    @Schema(description = "렌트 차량 고유 번호", example = "1")
    private long rentCarId;
    @Schema(description = "알림 내용", example = "렌트 차량이 도착했습니다. 확인해주세요!")
    private String contents;
}
