package com.drtaa.core_ui

import android.content.Context
import android.view.View
import android.view.inputmethod.InputMethodManager
import android.widget.Toast
import androidx.fragment.app.Fragment
import com.google.android.material.snackbar.Snackbar

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