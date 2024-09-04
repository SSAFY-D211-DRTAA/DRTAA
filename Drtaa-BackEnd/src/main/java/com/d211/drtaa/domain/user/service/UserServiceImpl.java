package com.d211.drtaa.domain.user.service;

import com.d211.drtaa.domain.common.service.JwtTokenService;
import com.d211.drtaa.domain.common.service.S3Service;
import com.d211.drtaa.domain.user.dto.request.FormLoginRequestDTO;
import com.d211.drtaa.domain.user.dto.request.SocialLoginRequestDTO;
import com.d211.drtaa.domain.user.dto.request.SocialSignUpRequestDTO;
import com.d211.drtaa.domain.user.dto.response.UserInfoResponseDTO;
import com.d211.drtaa.global.config.jwt.JwtToken;
import com.d211.drtaa.domain.user.entity.User;
import com.d211.drtaa.domain.user.repository.UserRepository;
import com.d211.drtaa.global.exception.user.UserNicknameDuplicateException;
import lombok.RequiredArgsConstructor;
import org.springframework.security.authentication.BadCredentialsException;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.stereotype.Service;
import org.springframework.web.multipart.MultipartFile;

@Service
@RequiredArgsConstructor
public class UserServiceImpl implements UserService{

    private final PasswordEncoder passwordEncoder;
    private final JwtTokenService jwtTokenService;
    private final CustomUserDetailsService userDetailsService;
    private final S3Service s3Service;
    private final UserRepository userRepository;

    @Override
    public JwtToken FormLogin(FormLoginRequestDTO request) {
        // 사용자 검증
        UserDetails userDetails = userDetailsService.loadUserByUsername(request.getUserProviderId());
        
        // 비밀번호 매칭 확인
        if (!passwordEncoder.matches(request.getUserPassword(), userDetails.getPassword())) 
            throw new BadCredentialsException("유효하지 않은 비밀번호입니다.");

        // JWT 토큰 발급
        JwtToken tokens = jwtTokenService.generateToken(request.getUserProviderId(), request.getUserPassword());

        return tokens;
    }

    @Override
    public JwtToken SocialLogin(SocialLoginRequestDTO request) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(request.getUserProviderId())
                .orElse(null);

        // 사용자가 없는 경우
        if (user == null) {
            SocialSignUpRequestDTO requestDTO = SocialSignUpRequestDTO.builder()
                    .userProviderId(request.getUserProviderId())
                    .userNickname(request.getUserNickname())
                    .userProfileImg(request.getUserProfileImg())
                    .userLogin(request.getUserLogin())
                    .build();

            userDetailsService.createUser(requestDTO, request.getUserProfileImg());
        } else {
            // 비밀번호 매칭 확인
            if (!passwordEncoder.matches("", user.getUserPassword()))
                throw new BadCredentialsException("유효하지 않은 비밀번호입니다.");
        }

        // JWT 토큰 발급
        JwtToken tokens = jwtTokenService.generateToken(request.getUserProviderId(), "");

        return tokens;
    }

    @Override
    public UserInfoResponseDTO info(String userProviderId) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        // 필요한 정보만 UserInfoResponseDTO로 편집해 반환
        UserInfoResponseDTO userInfo = UserInfoResponseDTO.builder()
                .userId(user.getUserId())
                .userNickname(user.getUserNickname())
                .userProfileImg(user.getUserProfileImg())
                .userLogin(user.getUserLogin())
                .userIsAdmin(user.isUserIsAdmin())
                .build();

        return userInfo;
    }

    @Override
    public void delete(String userProviderId) {
        userRepository.deleteByUserProviderId(userProviderId);
    }

    @Override
    public void updateImg(String userName, MultipartFile image) throws Exception {
        // 사용자 조회
        User user = userRepository.findByUserProviderId(userName)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));
        
        String beforeImg = user.getUserProfileImg(); // 사용자의 변경전 이미지
        String newImgUrl = null; // 사용자가 변경할 이미지
        if (image != null && !image.isEmpty()) {
            // 기존 이미지 S3에서 삭제
            s3Service.deleteS3(beforeImg);
            // 새 이미지 S3의 업로드
            newImgUrl = s3Service.uploadS3(image, "profileImg");
        }

        // 새 이미지 DB에 업데이트
        user.setUserProfileImg(newImgUrl);

        // DB 저장
        userRepository.save(user);
    }

    @Override
    public void deleteImg(String userName) throws Exception {
        // 사용자 조회
        User user = userRepository.findByUserProviderId(userName)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        String beforeImg = user.getUserProfileImg(); // 사용자의 변경전 이미지
        // S3에서 변경 전 이미지 삭제
        s3Service.deleteS3(beforeImg);

        // 기본 이미지 DB에 업데이트
        String basicImgUrl = "defualtProfileImg.png";  // 앱 내 기본 이미지
        user.setUserProfileImg(basicImgUrl);

        // DB 저장
        userRepository.save(user);
    }

    @Override
    public void updateNickname(String userName, String nickName) {
        if (!userRepository.existsByUserNickname(nickName)) {
            // 사용자 조회
            User user = userRepository.findByUserProviderId(userName)
                    .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

            // 새로운 닉네임 DB에 업데이트
            user.setUserNickname(nickName);

            // DB 저장
            userRepository.save(user);
        } else {
            new UserNicknameDuplicateException(nickName + "을 닉네임으로 가진 회원이 이미 존재합니다.");
        }
    }

    @Override
    public boolean chkUSerPorviderId(String userPorviderId) {
        return userRepository.existsByUserProviderId(userPorviderId);
    }
}
