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
    Optional<DatePlaces> findByDatePlacesId(Long datePlacesId);
    List<DatePlaces> findByTravelDates(TravelDates travelDates);
    @Query("SELECT dp FROM DatePlaces dp WHERE dp.travelDates.travelDatesId = :travelDatesId ORDER BY dp.datePlacesOrder DESC")
    Optional<DatePlaces> findLastPlaceByTravelDatesId(@Param("travelDatesId") Long travelDatesId);
    List<DatePlaces> findByTravelDatesAndDatePlacesOrderGreaterThan(TravelDates date, int datePlacesOrder);
    Optional<DatePlaces> findByTravelDatesAndDatePlacesOrder(TravelDates date, int datePlacesOrder);
    DatePlaces findFirstByTravelDatesOrderByDatePlacesOrderAsc(TravelDates travelDates);

    // delete
    void deleteAllByTravelAndTravelDates(Travel travel, TravelDates dates);

}
