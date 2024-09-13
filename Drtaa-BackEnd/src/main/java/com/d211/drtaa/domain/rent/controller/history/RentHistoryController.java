package com.d211.drtaa.domain.rent.controller.history;

import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequestMapping("/rent-history")
@RequiredArgsConstructor
@Log4j2
@Tag(name = "렌트기록 컨트롤러", description = "렌트 기록 관련 기능 수행")
public class RentHistoryController {
}
