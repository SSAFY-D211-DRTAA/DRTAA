package com.d211.drtaa.domain.travel.repository;

import com.d211.drtaa.domain.travel.entity.DatePlaces;
import com.d211.drtaa.domain.travel.entity.Travel;
import com.d211.drtaa.domain.travel.entity.TravelDates;
import io.lettuce.core.dynamic.annotation.Param;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.util.List;
import java.util.Optional;

public interface DatePlacesRepository extends JpaRepository<DatePlaces, Long> {
    // find
    Optional<DatePlaces> findByDatePlacesId(long datePlacesId);
    @Query(value = "SELECT * FROM date_places WHERE travel_dates_id = :travelDatesId", nativeQuery = true)
    List<DatePlaces> findByTravelDatesId(@Param("travelDatesId") Long travelDatesId);

    // delete
    void deleteAllByTravelAndTravelDates(Travel travel, TravelDates dates);
}
