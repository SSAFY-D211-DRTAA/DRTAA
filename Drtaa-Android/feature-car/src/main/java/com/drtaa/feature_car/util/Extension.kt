package com.drtaa.feature_car.util

import android.animation.ObjectAnimator
import android.graphics.drawable.GradientDrawable
import android.view.View
import com.drtaa.feature_car.CarFragment.Companion.DURATION
import com.drtaa.feature_car.CarFragment.Companion.HALF
import com.drtaa.feature_car.CarFragment.Companion.MAX_ROTATION
import com.drtaa.feature_car.CarFragment.Companion.OVERLAY_VIEW
import com.drtaa.feature_car.CarFragment.Companion.POINT_XY

fun View.resetOverlay() {
    this.animate()
        .alpha(0f)
        .setDuration(DURATION)
        .start()
}

fun View.resetReflection() {
    ObjectAnimator.ofFloat(this, "translationY", this.height.toFloat())
        .apply {
            duration = DURATION
            start()
        }
}

fun View.updateOverlay(percentX: Float, percentY: Float) {
    val gradient = this.background as GradientDrawable
    gradient.setGradientCenter(percentX, percentY)
    this.alpha = OVERLAY_VIEW
}

fun View.updateReflection(percentY: Float) {
    val translationY = (percentY - POINT_XY) * 2 * this.height
    ObjectAnimator.ofFloat(this, "translationY", translationY).apply {
        duration = 0
        start()
    }
}

fun View.rotateCard(v: View, x: Float, y: Float) {
    val (centerX, centerY) = v.width / HALF to v.height / HALF
    var rotateX = (centerY - y) / centerY * MAX_ROTATION
    var rotateY = (x - centerX) / centerX * MAX_ROTATION

    rotateX = rotateX.coerceIn(-MAX_ROTATION, MAX_ROTATION)
    rotateY = rotateY.coerceIn(-MAX_ROTATION, MAX_ROTATION)

    this.animate()
        .rotationX(rotateX)
        .rotationY(rotateY)
        .setDuration(0)
        .start()
}