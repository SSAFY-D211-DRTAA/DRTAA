package com.d211.study.exception.auth;

public class AuthenticationCreationException extends RuntimeException {
    public AuthenticationCreationException(String message) {
        super(message);
    }

    public AuthenticationCreationException(String message, Throwable cause) {
        super(message, cause);
    }
}