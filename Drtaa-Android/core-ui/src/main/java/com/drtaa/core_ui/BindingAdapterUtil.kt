package com.drtaa.core_ui

import android.widget.ImageView
import android.widget.TextView
import androidx.databinding.BindingAdapter
import com.bumptech.glide.Glide
import com.bumptech.glide.load.resource.bitmap.CenterCrop
import com.bumptech.glide.load.resource.bitmap.RoundedCorners
import java.util.Locale

const val RADIUS = 10

@BindingAdapter("app:setTextUppercase")
fun TextView.setTextUppercase(text: String?) {
    this.text = text?.uppercase(Locale.getDefault())
}

@BindingAdapter("app:setImgUrl")
fun ImageView.loadImageUrl(imgUrl: String) {
    Glide.with(context)
        .load(imgUrl)
        .fitCenter()
        .transform(CenterCrop(), RoundedCorners(RADIUS))
//        .placeholder()
//        .error()
//        .fallback()
        .into(this)

    this.adjustViewBounds = true
}
