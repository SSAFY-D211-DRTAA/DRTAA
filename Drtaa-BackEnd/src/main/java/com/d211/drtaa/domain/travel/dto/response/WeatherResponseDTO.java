package com.d211.drtaa.domain.travel.dto.response;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

@Data
@AllArgsConstructor
@Builder
public class WeatherResponseDTO {
    public WeatherResponseDTO(String dayOfWeek, int dayOfMonth) {
        this.dayOfWeek = dayOfWeek;
        this.dayOfMonth = dayOfMonth;
    }
    @Schema(description = "요일",example = "금")
    private String dayOfWeek; // 요일

    @Schema(description = "날짜", example = "1")
    private int dayOfMonth; // 날짜

    @Schema(description = "최저온도", example = "17.31")
    private double min; // 최저온도

    @Schema(description = "최대온도", example = "26.12")
    private double max; // 최대온도

    @Schema(description = "날씨설명", example = "04d")
    private String description; // 날씨설명

    @Schema(description = "체감온도", example = "19.705")
    private double feelsLike; // 체감온도

    @Schema(description = "습도", example = "85")
    private int humidity; // 습도

    @Schema(description = "강수확률", example = "0.32375")
    private double pop; // 강수확률
}
