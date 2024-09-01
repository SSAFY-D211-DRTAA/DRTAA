package com.d211.drtaa.user.service;

import com.d211.drtaa.user.dto.request.SignUpRequestDTO;
import com.d211.drtaa.user.entity.User;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.security.provisioning.UserDetailsManager;

public interface CustomUserDetailsService extends UserDetailsManager {
    // 회원 정보 로딩
    UserDetails loadUserByUsername(String userEmail) throws UsernameNotFoundException;

    // 회원 생성
    UserDetails createUserDetails(User user);
    void createUser(UserDetails user);
    void createUser(SignUpRequestDTO request);

    // 회원 삭제
    void deleteUser(String userName);

    // 회원 정보 수정
    void updateUser(UserDetails user);

    // 회원의 memberPssword(oldPassword)를 newPassword로 수정
    void changePassword(String oldPassword, String newPassword);

    // username을 사용해 회원 존재 확인
    boolean userExists(String userName);
}
