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
        .into(this)

    if (this !is de.hdodenhof.circleimageview.CircleImageView) {
        this.adjustViewBounds = true
    }
}

@BindingAdapter("app:setTextPrice")
fun TextView.setTextPrice(price: Int) {
    this.text = format(Locale.KOREA, "%,d원", price)
}

@BindingAdapter("app:setTextTime")
fun TextView.setTextTime(time: Double) {
    this.text = if (time - time.toInt() > 0) time.toString() else time.toInt().toString()
}
