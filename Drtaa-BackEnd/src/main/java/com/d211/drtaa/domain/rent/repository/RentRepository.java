package com.d211.drtaa.domain.rent.repository;

import com.d211.drtaa.domain.rent.entity.Rent;
import com.d211.drtaa.domain.rent.entity.RentStatus;
import com.d211.drtaa.domain.user.entity.User;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;

public interface RentRepository extends JpaRepository<Rent, Long> {
    // find
    List<Rent> findByUser(User user);
    Optional<Rent> findByRentId(Long rentId);
    @Query("SELECT r FROM Rent r " +
            "JOIN FETCH r.rentCar rc " +
            "WHERE r.rentStatus = 'in_progress' " +
            "AND r.user.userProviderId = :userProviderId " +
            "AND r.rentStartTime <= CURRENT_TIMESTAMP " +
            "AND r.rentEndTime > CURRENT_TIMESTAMP")
    Optional<Rent> findCurrentRentByUserProviderId(String userProviderId);
    @Query("SELECT r FROM Rent r " +
            "WHERE r.user = :user AND r.rentStatus = 'completed'")
    List<Rent> findByUserAndRentStatusCompleted(User user);
    List<Rent> findByUserAndRentStatusInOrderByRentStatusDesc(User user, List<RentStatus> rentStatuses);
    List<Rent> findByRentStartTimeBetween(LocalDateTime startDate, LocalDateTime endDate);
    Optional<Rent> findFirstByRentCar_RentCarIdAndRentStatusAndRentStartTimeLessThanEqualAndRentEndTimeGreaterThanEqual(
            Long rentCarId,
            RentStatus rentStatus,
            LocalDateTime rentStartTime,
            LocalDateTime rentEndTime
    );



    // exists
    boolean existsByUserAndRentStatusAndRentStartTimeBetweenOrRentEndTimeBetween(User user, RentStatus rentStatus, LocalDateTime localDateTime, LocalDateTime localDateTime1, LocalDateTime localDateTime2, LocalDateTime localDateTime3);

}
