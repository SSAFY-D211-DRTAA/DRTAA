package com.d211.drtaa.domain.travel.controller;

import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequestMapping("/travel")
@RequiredArgsConstructor
@Log4j2
@Tag(name = "여행 컨트롤러", description = "여행 일정 관련 기능 수행")
public class TravelController {
}
