package com.drtaa.core_ui

import android.content.Context
import android.widget.ImageView
import com.bumptech.glide.Glide

fun ImageView.centerCrop(url: String, context: Context) {
    Glide.with(context)
        .load(url)
        .centerCrop()
        .into(this)
}

fun ImageView.circleCrop(url: String, context: Context) {
    Glide.with(context)
        .load(url)
        .circleCrop()
        .into(this)
}