package com.drtaa.core_ui

import android.widget.ImageView
import android.widget.TextView
import androidx.databinding.BindingAdapter
import com.bumptech.glide.Glide
import java.lang.String.format
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
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

@BindingAdapter("paymentProduct")
fun TextView.setPaymentProduct(orderId: String) {
    text = when (orderId) {
        "1" -> "DRTAA 렌트"
        "2" -> "DRTAA 택시"
        else -> "DRTAA 서비스"
    }
}

@BindingAdapter("paymentMethod")
fun TextView.setPaymentMethod(method: String) {
    text = method
    val color = when (method) {
        "네이버페이" -> R.color.naver_green
        "토스" -> R.color.toss_blue
        "카카오페이" -> R.color.kakao_yellow
        else -> R.color.default_text_color
    }
    setTextColor(color)
}

@BindingAdapter("rentTime")
fun TextView.setRentStartTime(time: String) {
    text = LocalDateTime.parse(time).format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm"))
}

@BindingAdapter("paymentImage")
fun ImageView.setPaymentImage(orderId: String) {
    setImageResource(
        when (orderId) {
            "1" -> R.drawable.ic_rent_payment
            else -> R.drawable.ic_taxi_payment
        }
    )
}
