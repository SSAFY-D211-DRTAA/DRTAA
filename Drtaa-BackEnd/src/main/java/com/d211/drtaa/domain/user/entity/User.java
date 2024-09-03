package com.d211.drtaa.domain.user.entity;

import io.swagger.v3.oas.annotations.media.Schema;
import jakarta.persistence.*;
import lombok.*;
import org.springframework.security.core.GrantedAuthority;
import org.springframework.security.core.authority.SimpleGrantedAuthority;
import org.springframework.security.core.userdetails.UserDetails;

import java.util.Collection;
import java.util.Collections;

@Entity
@Table(name = "user")
@NoArgsConstructor
@AllArgsConstructor
@Getter
@Setter
@Builder
public class User implements UserDetails {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "user_id", nullable = false)
    @Schema(description = "회원 고유 번호", example = "1")
    private long userId;

    @Column(name = "user_provider_id", nullable = false)
    @Schema(description = "회원 고유 번호", example = "1")
    private String userProviderId;

    @Column(name = "user_email", nullable = false)
    @Schema(description = "회원 ID", example = "test@naver.com")
    private String userEmail;

    @Column(name = "user_password", nullable = false)
    @Schema(description = "회원 PW", example = "암호화된 내용")
    private String userPassword;

    @Column(name = "user_nickname", nullable = false)
    @Schema(description = "회원 닉네임", example = "TEST")
    private String userNickname;

    @Column(name = "user_refreshtoken", nullable = false)
    @Schema(description = "회원 refreshToken", example = "ex7534487435468~~")
    private String userRefreshToken;

    @Column(name = "user_login", nullable = false)
    @Schema(description = "회원 로그인처", example = "Form")
    private String userLogin;

    @Column(name = "user_is_admin", nullable = false)
    @Schema(description = "회원 관리자 유무", example = "false")
    private boolean userIsAdmin;


    @Override
    public Collection<? extends GrantedAuthority> getAuthorities() {
        return Collections.singletonList(new SimpleGrantedAuthority(userIsAdmin ? "ROLE_ADMIN" : "ROLE_USER"));
    }

    @Override
    public String getPassword() {
        return userPassword;
    }

    @Override
    public String getUsername() {
        return userEmail;
    }

    @Override
    public boolean isAccountNonExpired() {
        return true;
    }

    @Override
    public boolean isAccountNonLocked() {
        return true;
    }

    @Override
    public boolean isCredentialsNonExpired() {
        return true;
    }

    @Override
    public boolean isEnabled() {
        return true;
    }
}
