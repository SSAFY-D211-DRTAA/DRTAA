package com.d211.drtaa.global.config.websocket;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@JsonIgnoreProperties(ignoreUnknown = true)
@NoArgsConstructor
@AllArgsConstructor
@Getter
@Setter
public class MyMessage {
    private String action;
    private double latitude;
    private double longitude;
    private long rentCarId;
    private String destinationName;

    public MyMessage(String action) {
        this.action = action;
    }

    public MyMessage(String action, long rentCarId) {
        this.action = action;
        this.rentCarId = rentCarId;
    }

    public MyMessage(String action, double latitude, double longitude, long rentCarId) {
        this.action = action;
        this.longitude = longitude;
        this.latitude = latitude;
        this.rentCarId = rentCarId;
    }
}