package com.d211.drtaa.global.exception.user;

public class UserCreationException extends RuntimeException {
    public UserCreationException(String message) {
        super(message);
    }

    public UserCreationException(String message, Throwable cause) {
        super(message, cause);
    }
}