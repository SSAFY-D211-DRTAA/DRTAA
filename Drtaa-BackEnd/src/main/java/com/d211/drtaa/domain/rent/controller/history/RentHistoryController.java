package com.d211.drtaa.domain.rent.controller.history;

import com.d211.drtaa.domain.rent.dto.response.UserDetailHistoryResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.UserHistoryResponseDTO;
import com.d211.drtaa.domain.rent.entity.history.RentHistory;
import com.d211.drtaa.domain.rent.repository.history.RentHistoryRepository;
import com.d211.drtaa.domain.rent.service.history.RentHistoryService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.Authentication;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequestMapping("/rent-history")
@RequiredArgsConstructor
@Log4j2
@Tag(name = "렌트기록 컨트롤러", description = "렌트 기록 관련 기능 수행")
public class RentHistoryController {

    private final RentHistoryService rentHistoryService;
    
    @GetMapping
    @Operation(summary = "렌트 기록 조회", description = "사용자의 전체 렌트 기록을 조회")
    public ResponseEntity myHistory(Authentication authentication) {
        try{
            UserHistoryResponseDTO responseDTO = rentHistoryService.getHistory(authentication.getName());
            
            return ResponseEntity.ok().body(responseDTO);
        } catch (Exception e) {
            return ResponseEntity.badRequest().body(e.getMessage());
        }
    }

    @GetMapping("/{rentHistoryId}")
    @Operation(summary = "렌트 기록 상세 조회", description = "사용자의 렌트 기록 상세 조회")
    public ResponseEntity myDetailHistory(@PathVariable("rentHistoryId") Long rentHistoryId) {
        try {
            UserDetailHistoryResponseDTO responseDTO = rentHistoryService.getDetailHistory(rentHistoryId);

            return ResponseEntity.ok().body(responseDTO);
        } catch (Exception e) {
            return ResponseEntity.badRequest().body(e.getMessage());
        }
    }
}
