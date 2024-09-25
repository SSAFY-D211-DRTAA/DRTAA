package com.d211.drtaa.global.util.fcm;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;

import javax.management.Notification;

@Builder
@AllArgsConstructor
@Getter
public class FcmMessage {

    private boolean validateOnly;
    private Message message;

    @Builder
    @AllArgsConstructor
    @Getter
    public static class Message {
        private Notification notification;
        private String token;
        private FcmDTO data; //FcmDTO
    }

    @Builder
    @AllArgsConstructor
    @Getter
    public static class Notification {
        private String title; //FcmDTO것과동일
        private String body;  //FcmDTO것과동일
        private String image;
    }

    @Builder
    @AllArgsConstructor
    @Getter
    public static class FcmDTO { //FcmDTO
        private String title;
        private String body;
    }
}
