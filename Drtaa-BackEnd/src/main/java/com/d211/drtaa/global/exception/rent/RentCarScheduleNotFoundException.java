package com.d211.drtaa.global.exception.rent;

public class RentCarScheduleNotFoundException extends RuntimeException {
    public RentCarScheduleNotFoundException(String message) {
        super(message);
    }

    public RentCarScheduleNotFoundException(String message, Throwable cause) {
        super(message, cause);
    }
}
