package com.drtaa.core_ui

import android.widget.TextView
import androidx.databinding.BindingAdapter
import java.util.Locale

object BindingAdapterUtil {
    @JvmStatic
    @BindingAdapter("app:setTextUppercase")
    fun setTextUppercase(view: TextView, text: String?) {
        view.text = text?.uppercase(Locale.getDefault())
    }
}