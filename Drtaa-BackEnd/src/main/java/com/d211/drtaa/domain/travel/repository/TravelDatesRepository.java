package com.d211.drtaa.domain.travel.repository;

import com.d211.drtaa.domain.travel.entity.Travel;
import com.d211.drtaa.domain.travel.entity.TravelDates;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.List;
import java.util.Optional;

public interface TravelDatesRepository extends JpaRepository<TravelDates, Long> {
    List<TravelDates> findByTravel(Travel travel);
    Optional<TravelDates> findByTravelDatesId(Long travelId);
    Optional<TravelDates> findFirstByTravelOrderByTravelDatesDateAsc(Travel travel);
}