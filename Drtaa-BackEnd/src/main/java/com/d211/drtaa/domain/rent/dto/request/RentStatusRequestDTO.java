package com.d211.drtaa.domain.rent.dto.request;

import com.d211.drtaa.domain.rent.entity.RentStatus;
import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

@Data
@AllArgsConstructor
@Getter
public class RentStatusRequestDTO {
    @Schema(description = "렌트 고유 번호", example = "1")
    private long rentId;
    @Schema(description = "렌트 상태", example = "RESERVED")
    private RentStatus rentStatus;
}
