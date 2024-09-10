package com.drtaa.core_ui

fun checkValidPassword(password: String): Boolean {
    val passwordRegex = Regex("^(?=.*[a-zA-Z0-9])(?=.*[\\W_])[a-zA-Z0-9\\W_]{8,20}\$")
    return password.matches(passwordRegex)
}