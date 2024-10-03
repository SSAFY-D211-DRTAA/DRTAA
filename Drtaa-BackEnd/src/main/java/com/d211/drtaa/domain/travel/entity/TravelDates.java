package com.d211.drtaa.domain.travel.entity;

import io.swagger.v3.oas.annotations.media.Schema;
import jakarta.persistence.*;
import lombok.*;
import org.hibernate.annotations.ColumnDefault;

import java.time.LocalDate;
import java.util.List;

@Entity
@Table(name = "travel_dates")
@NoArgsConstructor
@AllArgsConstructor
@Getter
@Setter
@Builder
public class TravelDates {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "travel_dates_id", nullable = false)
    @Schema(description = "여행 일정 고유번호", example = "1")
    private Long travelDatesId;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "travel_id", nullable = false)
    @Schema(description = "여행 고유번호", example = "1")
    private Travel travel;

    @Column(name = "travel_dates_date", nullable = false)
    @Schema(description = "여행 일정 날짜", example = "2024/01/01")
    private LocalDate travelDatesDate;

    @Column(name = "travel_dates_is_expired", nullable = false)
    @ColumnDefault("0")
    @Schema(description = "여행 일정 만료 여부", example = "false")
    private Boolean travelDatesIsExpired;

    @OneToMany(mappedBy = "travelDates", cascade = CascadeType.ALL)
    private List<DatePlaces> places;
}
