package com.d211.drtaa.domain.rent.repository.history;

import com.d211.drtaa.domain.rent.entity.history.RentHistory;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface RentHistoryRepository extends JpaRepository<RentHistory, Long> {
    // find
    Optional<RentHistory> findByUser_UserId(Long userId);
    Optional<RentHistory> findByRentHistoryId(Long rentHistoryId);
}
