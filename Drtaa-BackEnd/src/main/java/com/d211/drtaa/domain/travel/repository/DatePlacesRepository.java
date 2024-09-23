package com.d211.drtaa.domain.travel.repository;

import com.d211.drtaa.domain.travel.entity.DatePlaces;
import io.lettuce.core.dynamic.annotation.Param;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.util.List;

public interface DatePlacesRepository extends JpaRepository<DatePlaces, Long> {
    @Query(value = "SELECT * FROM date_places WHERE travel_dates_id = :travelDatesId", nativeQuery = true)
    List<DatePlaces> findByTravelDatesId(@Param("travelDatesId") Long travelDatesId);
}