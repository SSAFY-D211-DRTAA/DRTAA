package com.d211.drtaa.domain.test;

import com.d211.drtaa.global.config.websocket.MyMessage;
import com.d211.drtaa.global.config.websocket.WebSocketConfig;
import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.client.standard.StandardWebSocketClient;
import org.springframework.web.socket.handler.TextWebSocketHandler;

@RestController
@RequiredArgsConstructor
@Slf4j
@RequestMapping("/test")
public class TestController {

//    @GetMapping("/test")
//    public String test() {
//        return "GitLab - Jenkins - MM Webhook 테스트 중";
//    }
//
//    @GetMapping("/re")
//    public String home() {
//        return "redirect:/index.html"; // static 폴더의 index.html로 리다이렉트
//    }

    private final WebSocketConfig webSocketConfig;
    private final ObjectMapper objectMapper = new ObjectMapper();

    @GetMapping
    public String sendVehicleDispatch() {
        try {
            StandardWebSocketClient client = new StandardWebSocketClient();
            WebSocketSession session = client.execute(new TextWebSocketHandler() {
                @Override
                protected void handleTextMessage(WebSocketSession session, TextMessage message) {
                    // 서버로부터 받은 메시지를 처리하는 로직
                    log.info(message.getPayload());
                }
            }, webSocketConfig.getUrl()).get();

            MyMessage message = new MyMessage("vehicle_dispatch");
            String jsonMessage = objectMapper.writeValueAsString(message);
            session.sendMessage(new TextMessage(jsonMessage));

            log.info("Sent message: {}", jsonMessage);
            return "Vehicle dispatch message sent successfully";
        } catch (Exception e) {
            log.error("Error sending vehicle dispatch message", e);
            return "Failed to send message";
        }
    }
}
