package com.d211.drtaa.global.exception.travel;

public class TravelAllPlacesVisitedException extends RuntimeException {
    public TravelAllPlacesVisitedException(String message) {
        super(message);
    }
    public TravelAllPlacesVisitedException(String message, Throwable cause) {
        super(message, cause);
    }
}
