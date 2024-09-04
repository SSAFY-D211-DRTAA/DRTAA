package com.d211.drtaa.domain.user.repository;

import com.d211.drtaa.domain.user.entity.User;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface UserRepository extends JpaRepository<User, Long> {
    // find
    Optional<User> findByUserProviderId(String userProviderId);
    Optional<String> findUserRefreshTokenByUserProviderId(String userProviderId);

    // exists
    boolean existsByUserProviderId(String userProviderId);
    boolean existsByUserNickname(String userNickname);

    // delete
    boolean deleteByUserProviderId(String userProviderId);
}
