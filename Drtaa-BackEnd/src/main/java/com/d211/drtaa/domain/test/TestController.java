package com.d211.drtaa.domain.test;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@Slf4j
@RequestMapping("/testt")
public class TestController {

    @GetMapping("/test")
    public String test() {
        return "GitLab - Jenkins - MM Webhook 테스트 중";
    }

    @GetMapping("/re")
    public String home() {
        return "redirect:/index.html"; // static 폴더의 index.html로 리다이렉트
    }
}
