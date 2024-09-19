package com.drtaa.core_ui

import android.widget.ImageView
import android.widget.TextView
import androidx.databinding.BindingAdapter
import com.bumptech.glide.Glide
import java.lang.String.format
import java.util.Locale

const val RADIUS = 10

@BindingAdapter("app:setTextUppercase")
fun TextView.setTextUppercase(text: String?) {
    this.text = text?.uppercase(Locale.getDefault())
}

@BindingAdapter("app:setImgUrl")
fun ImageView.loadImageUrl(imgUrl: String?) {
    if (imgUrl == null) {
        return
    }

    Glide.with(context)
        .load(imgUrl)
//        .placeholder()
//        .error()
//        .fallback()
        .into(this)

    this.adjustViewBounds = true
}

@BindingAdapter("app:setTextPrice")
fun TextView.setTextPrice(price: Int) {
    this.text = format(Locale.KOREA, "%,d원", price)
}
