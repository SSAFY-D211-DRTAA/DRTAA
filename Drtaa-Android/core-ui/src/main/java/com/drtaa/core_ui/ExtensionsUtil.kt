package com.drtaa.core_ui

import android.view.View
import androidx.fragment.app.Fragment
import com.google.android.material.snackbar.Snackbar

fun Fragment.showSnackBar(message: String, view: View = requireView()) {
    Snackbar.make(view, message, Snackbar.LENGTH_SHORT).show()
}