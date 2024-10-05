package com.drtaa.core_ui

import android.view.View
import android.widget.ImageView

const val DURATION = 300L
const val ROTATION_90 = 90f
const val ROTATION_270 = 270f

fun foldLayout(imageBtn: ImageView, layout: View) {
    layout.visibility = View.GONE
    imageBtn.animate().apply {
        duration = DURATION
        rotation(ROTATION_90)
    }
}

fun expandLayout(imageBtn: ImageView, layout: View) {
    layout.visibility = View.VISIBLE
    imageBtn.animate().apply {
        duration = DURATION
        rotation(ROTATION_270)
    }
}