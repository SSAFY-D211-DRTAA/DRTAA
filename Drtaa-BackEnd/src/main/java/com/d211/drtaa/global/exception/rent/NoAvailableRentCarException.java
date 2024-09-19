package com.d211.drtaa.global.exception.rent;

public class NoAvailableRentCarException extends RuntimeException {
    public NoAvailableRentCarException(String message) {
        super(message);
    }

    public NoAvailableRentCarException(String message, Throwable cause) {
        super(message, cause);
    }
}