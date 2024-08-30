package com.d211.drtaa.user.repository;

import com.d211.drtaa.user.entity.User;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface UserRepository extends JpaRepository<User, Long> {
    // find
    Optional<User> findByUserEmail(String userEmail);
    Optional<String> findByUserRefreshTokenByUserEmail(String userEmail);

    // exists
    boolean existsByUserEmail(String userEmail);

    // delete
    boolean deleteByUserEmail(String userEmail);
}
