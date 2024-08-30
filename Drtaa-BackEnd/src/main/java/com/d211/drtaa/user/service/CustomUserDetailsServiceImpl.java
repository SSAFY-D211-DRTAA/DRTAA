package com.d211.drtaa.user.service;

import com.d211.drtaa.exception.user.UserCreationException;
import com.d211.drtaa.user.dto.request.SignUpRequestDTO;
import com.d211.drtaa.user.entity.User;
import com.d211.drtaa.user.repository.UserRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.annotation.Lazy;
import org.springframework.dao.DataIntegrityViolationException;
import org.springframework.security.access.AccessDeniedException;
import org.springframework.security.authentication.BadCredentialsException;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.context.SecurityContextHolder;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.security.provisioning.UserDetailsManager;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class CustomUserDetailsServiceImpl implements UserDetailsManager {

    @Autowired
    private final UserRepository userRepository;

    @Lazy
    @Autowired
    private PasswordEncoder passwordEncoder;

    @Override
    public UserDetails loadUserByUsername(String userEmail) throws UsernameNotFoundException {
        // userEmail으로 사용자를 찾아 UserDetails객체를 반환
        return userRepository.findByUserEmail(userEmail)
                .map(this::createUserDetails)
                .orElseThrow(() -> new UsernameNotFoundException("해당 회원을 찾을 수 없습니다."));
    }

    private UserDetails createUserDetails(User user) {
        // UserDetails객체 생성
        return org.springframework.security.core.userdetails.User.builder()
                .username(user.getUserEmail())
                .password(user.getUserPassword())
                .roles(user.isUserIsAdmin() ? "ADMIN" : "USER")
                .build();
    }

    @Override
    public void createUser(UserDetails user) {
        try {
            // 사용자가 이미 존재하는 경우
            if (userExists(user.getUsername()))
                throw new UserCreationException("사용자 이름을 가진 사용자가 이미 존재합니다." + user.getUsername());

            User newUser = (User) user;

            userRepository.save(newUser);
        } catch (DataIntegrityViolationException e) {
            throw new UserCreationException("사용자를 데이터베이스에 저장하는 중 오류가 발생했습니다.", e);
        } catch (Exception e) {
            throw new UserCreationException("사용자 생성 중에 예기치 않은 오류가 발생했습니다.", e);
        }
    }

    public void createUser(SignUpRequestDTO request) {
        User member = User.builder()
                .userEmail(request.getUserEmail())
                .userPassword(passwordEncoder.encode(request.getUserPassword()))
                .userNickname(request.getUserNickname())
                .userRefreshToken("")
                .userIsSocial(request.isUserIsSocial())
                .userIsAdmin(request.isUserIsAdmin())
                .build();

        createUser(member); // 기존 createUser(UserDetails user) 호출
    }

    @Override
    public void deleteUser(String username) {
        userRepository.deleteByUserEmail(username);
    }

    @Override
    public void updateUser(UserDetails user) {
        userRepository.findByUserEmail(user.getUsername())
                .ifPresent(existingUser  -> {
                    existingUser .setUserPassword(passwordEncoder.encode(user.getPassword()));
                    existingUser .setUserIsAdmin(user.getAuthorities().stream()
                            // user가 가진 권한들 중 하나라도 "ROLE_ADMIN"이라는 권한이 있으면 true를 반환
                            .anyMatch(a -> a.getAuthority().equals("ROLE_ADMIN")));
                    userRepository.save(existingUser);
                });
    }

    @Override
    public void changePassword(String oldPassword, String newPassword) {
        // 현재 인증된 사용자의 비밀번호를 변경
        Authentication currentUser = SecurityContextHolder.getContext().getAuthentication();
        if (currentUser == null)
            throw new AccessDeniedException("현재 사용자에 대한 컨텍스트에서 인증 개체를 찾을 수 없으므로 비밀번호를 변경할 수 없습니다.");

        String username = currentUser.getName();

        userRepository.findByUserEmail(username)
                .ifPresent(user -> {
                    if (passwordEncoder.matches(oldPassword, user.getUserPassword())) {
                        user.setUserPassword(passwordEncoder.encode(newPassword));
                        userRepository.save(user);
                    } else {
                        throw new BadCredentialsException("이전 비밀번호가 올바르지 않습니다.");
                    }
                });
    }

    @Override
    public boolean userExists(String username) {
        // 회원 존재 여부 확인
        return userRepository.existsByUserEmail(username);
    }
}

