package com.drtaa.core_ui

import android.content.Context
import android.content.res.Resources
import android.view.View
import android.view.inputmethod.InputMethodManager
import android.widget.Toast
import androidx.fragment.app.Fragment
import com.google.android.material.snackbar.Snackbar
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
import java.util.Locale

fun Fragment.showSnackBar(message: String, view: View = requireView()) {
    Snackbar.make(view, message, Snackbar.LENGTH_SHORT).show()
}

/**
 * 키보드 내리기
 */
fun Context.hideKeyboard(view: View) {
    val imm: InputMethodManager =
        getSystemService(Context.INPUT_METHOD_SERVICE) as InputMethodManager
    imm.hideSoftInputFromWindow(view.windowToken, 0)
}

fun Context.showToast(message: String) {
    Toast.makeText(this, message, Toast.LENGTH_SHORT).show()
}

fun Boolean.doOnTrue(func: () -> Unit): Boolean {
    if (this) {
        func()
    }
    return this
}

fun Boolean.doOnFalse(func: () -> Unit): Boolean {
    if (this.not()) {
        func()
    }
    return this
}

fun String.parseLocalDateTime(): String {
    val dateTime = LocalDateTime.parse(this)
    val formatter = DateTimeFormatter.ofPattern("yyyy년 M월 d일\na h시 m분", Locale.KOREAN)
    val formatted = dateTime.format(formatter)
    return formatted.replace("AM", "오전").replace("PM", "오후")
}

/**
 * dp 값을 px 값으로 변환하는 확장 함수
 */
fun Int.dpToPx(): Int {
    return (this * Resources.getSystem().displayMetrics.density).toInt()
}