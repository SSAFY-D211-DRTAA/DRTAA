package com.d211.study.repository;

import com.d211.study.domain.Member;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface MemberRepository extends JpaRepository<Member, Long> {
    Optional<Member> findByMemberUsername(String memberUsername);
    Optional<String> findMemberRefreshTokenByMemberUsername(String memberUsername);
    boolean existsByMemberUsername(String memberUsername);
    boolean deleteByMemberUsername(String memberUsername);
}
