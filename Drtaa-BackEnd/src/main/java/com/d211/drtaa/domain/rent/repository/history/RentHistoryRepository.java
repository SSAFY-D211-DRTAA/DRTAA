package com.d211.drtaa.domain.rent.repository.history;

import com.d211.drtaa.domain.rent.entity.history.RentHistory;
import com.d211.drtaa.domain.user.entity.User;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.List;
import java.util.Optional;

public interface RentHistoryRepository extends JpaRepository<RentHistory, Long> {
    // find
    List<RentHistory> findByUser(User user);
    Optional<RentHistory> findByRentHistoryId(Long rentHistoryId);
}
