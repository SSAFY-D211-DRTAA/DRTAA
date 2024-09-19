package com.d211.drtaa.global.exception.rent;

import lombok.Getter;

@Getter
public class RentHistoryNotFoundException extends RuntimeException {
    public RentHistoryNotFoundException(String message) {
        super(message);
    }

    public RentHistoryNotFoundException(String message, Throwable cause) {
        super(message, cause);
    }
}