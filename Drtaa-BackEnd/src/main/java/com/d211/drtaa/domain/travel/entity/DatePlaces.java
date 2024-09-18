package com.d211.drtaa.domain.travel.entity;

import io.swagger.v3.oas.annotations.media.Schema;
import jakarta.persistence.*;
import lombok.*;

import java.time.LocalDate;

@Entity
@Table(name = "date_places")
@NoArgsConstructor
@AllArgsConstructor
@Getter
@Setter
@Builder
public class DatePlaces {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "date_places_id", nullable = false)
    @Schema(description = "일정 장소 고유번호", example = "1")
    private long travelDatesId;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "travel_dates_id", nullable = false)
    @Schema(description = "여행 일정 고유번호", example = "1")
    private TravelDates travelDates;

    @Column(name = "date_places_name", nullable = false)
    @Schema(description = "일정 장소 이름", example = "디지털미디어시티역")
    private String datePlacesName;

    @Column(name = "date_places_lat", nullable = false)
    @Schema(description = "일정 장소 위도", example = "0.0")
    private double datePlacesLat;

    @Column(name = "date_places_lon", nullable = false)
    @Schema(description = "일정 장소 경도", example = "0.0")
    private double datePlacesLon;
}
