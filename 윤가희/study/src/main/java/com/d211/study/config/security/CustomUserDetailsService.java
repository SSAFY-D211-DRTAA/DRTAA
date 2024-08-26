package com.d211.study.config.security;


import com.d211.study.domain.Member;
import com.d211.study.dto.request.SignUpUserRequest;
import com.d211.study.exception.UserCreationException;
import com.d211.study.repository.MemberRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.beans.factory.annotation.Autowired;
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
public class CustomUserDetailsService implements UserDetailsManager {

    @Autowired
    private final MemberRepository memberRepository;
    @Autowired
    private PasswordEncoder passwordEncoder;

    @Override
    public UserDetails loadUserByUsername(String memberUsername) throws UsernameNotFoundException {
        // 사용자 이름으로 사용자를 찾아 UserDetails객체를 반환
        return memberRepository.findByMemberUsername(memberUsername)
                .map(this::createUserDetails)
                .orElseThrow(() -> new UsernameNotFoundException("해당 회원을 찾을 수 없습니다."));
    }

    private UserDetails createUserDetails(Member member) {
        return org.springframework.security.core.userdetails.User.builder()
                .username(member.getMemberUsername())
                .password(member.getMemberPassword())
                .roles(member.isMemberIsAdmin() ? "ADMIN" : "USER")
                .build();
    }

    @Override
    public void createUser(UserDetails user) {
        try {
            if (userExists(user.getUsername()))
                throw new UserCreationException("사용자 이름을 가진 사용자가 이미 존재합니다." + user.getUsername());

            Member member = (Member) user;

            memberRepository.save(member);
        } catch (DataIntegrityViolationException e) {
            throw new UserCreationException("사용자를 데이터베이스에 저장하는 중 오류가 발생했습니다.", e);
        } catch (Exception e) {
            throw new UserCreationException("사용자 생성 중에 예기치 않은 오류가 발생했습니다.", e);
        }
    }

    public void createUser(SignUpUserRequest request) {
        Member member = Member.builder()
                .memberUsername(request.getMemberUsername())
                .memberEmail(request.getMemberEmail())
                .memberPassword(passwordEncoder.encode(request.getMemberPassword()))
                .memberIsAdmin(request.isMemberIsAdmin())
                .memberRefreshToken("")
                .build();

        createUser(member); // 기존 createUser(UserDetails user) 호출
    }

    @Override
    public void deleteUser(String username) {
        memberRepository.deleteByMemberUsername(username);
    }

    @Override
    public void updateUser(UserDetails user) {
        memberRepository.findByMemberUsername(user.getUsername())
                .ifPresent(member -> {
                    member.setMemberPassword(passwordEncoder.encode(user.getPassword()));
                    member.setMemberIsAdmin(user.getAuthorities().stream()
                            .anyMatch(a -> a.getAuthority().equals("ROLE_ADMIN")));
                    memberRepository.save(member);
                });
    }

    @Override
    public void changePassword(String oldPassword, String newPassword) {
        // 현재 인증된 사용자의 비밀번호를 변경합니다.
        Authentication currentUser = SecurityContextHolder.getContext().getAuthentication();
        if (currentUser == null) {
            throw new AccessDeniedException("현재 사용자에 대한 컨텍스트에서 인증 개체를 찾을 수 없으므로 비밀번호를 변경할 수 없습니다.");
        }

        String username = currentUser.getName();

        memberRepository.findByMemberUsername(username)
                .ifPresent(member -> {
                    if (passwordEncoder.matches(oldPassword, member.getMemberPassword())) {
                        member.setMemberPassword(passwordEncoder.encode(newPassword));
                        memberRepository.save(member);
                    } else {
                        throw new BadCredentialsException("이전 비밀번호가 올바르지 않습니다.");
                    }
                });
    }

    @Override
    public boolean userExists(String username) {
        return memberRepository.existsByMemberUsername(username);
    }

}
