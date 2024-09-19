package com.d211.drtaa.domain.travel.entity;

import io.swagger.v3.oas.annotations.media.Schema;
import jakarta.persistence.*;
import lombok.*;

import java.time.LocalDate;

@Entity
@Table(name = "travel")
@NoArgsConstructor
@AllArgsConstructor
@Getter
@Setter
@Builder
public class Travel {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "travel_id", nullable = false)
    @Schema(description = "여행 고유 번호", example = "1")
    private Long travelId;

    @Column(name = "travel_name", nullable = false)
    @Schema(description = "여행 이름", example = "서울 여행")
    private String travelName;
    
    @Column(name = "travel_start_date", nullable = false)
    @Schema(description = "여행 시작 일정", example = "2024/01/01")
    private LocalDate travelStartDate;

    @Column(name = "travel_end_date", nullable = false)
    @Schema(description = "여행 종료 일정", example = "2024/01/02")
    private LocalDate travelEndDate;
}
