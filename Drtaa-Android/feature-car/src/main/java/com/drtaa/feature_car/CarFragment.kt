package com.drtaa.feature_car

import android.animation.ObjectAnimator
import android.annotation.SuppressLint
import android.graphics.drawable.GradientDrawable
import android.view.MotionEvent
import android.view.View
import android.widget.ImageView
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_car.databinding.FragmentCarBinding
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class CarFragment : BaseFragment<FragmentCarBinding>(R.layout.fragment_car) {
    private lateinit var cardView: View
    private lateinit var overlayView: View
    private lateinit var reflectionView: View
    private lateinit var cardImage: ImageView
    private var isTouching: Boolean = false
    private var isNeko: Boolean = true
    private var touchStartTime: Long = 0
    private val touchPressTime = 100

    override fun initView() {
        binding.apply {
            cardView = cvTourCard
            overlayView = viewTourOverlay
            reflectionView = viewTourReflection
            cardImage = ivTourCard
        }
        setupCardTouchListener()
    }

    @SuppressLint("ClickableViewAccessibility")
    private fun setupCardTouchListener() {
        cardView.setOnTouchListener { v, event ->
            when (event.action) {
                MotionEvent.ACTION_DOWN -> { // 처음 눌렀을 때 시간을 재서 클릭과 드로잉을 구분해야지
                    isTouching = true
                    touchStartTime = System.currentTimeMillis()
                    true
                }
                MotionEvent.ACTION_MOVE -> {  // 이동했을 때
                    if (isTouching && System.currentTimeMillis() - touchStartTime > touchPressTime) {
                        val x = event.x
                        val y = event.y

                        val centerX = v.width / 2f
                        val centerY = v.height / 2f

                        val rotateX = (centerY - y) / centerY * 10
                        val rotateY = (x - centerX) / centerX * 10

                        cardView.animate()
                            .rotationX(rotateX)
                            .rotationY(rotateY)
                            .setDuration(0)
                            .start()

                        updateOverlay(x / v.width, y / v.height)
                        updateReflection(y / v.height)
                    }
                    true
                }
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    if (isTouching && System.currentTimeMillis() - touchStartTime <= touchPressTime) {
                        isNeko = !isNeko // true false 전환
                        val newImageResource = if (isNeko) R.drawable.kanna else R.drawable.kanna //이미지 전환, 나중에는 view 전환으로 하면 될 듯?
                        cardImage.setImageResource(newImageResource) // 칸나칸나 이미지 적용
                    }

                    isTouching = false
                    cardView.animate()
                        .rotationX(0f)
                        .rotationY(0f)
                        .setDuration(300)
                        .start()

                    resetOverlay()
                    resetReflection()
                    true
                }
                else -> false
            }
        }
    }

    private fun updateOverlay(percentX: Float, percentY: Float) {
        val gradient = overlayView.background as GradientDrawable
        gradient.setGradientCenter(percentX, percentY)
        overlayView.alpha = 0.6f
    }

    private fun resetOverlay() {
        overlayView.animate()
            .alpha(0f)
            .setDuration(300)
            .start()
    }

    private fun updateReflection(percentY: Float) {
        val translationY = (percentY - 0.5f) * 2 * reflectionView.height
        ObjectAnimator.ofFloat(reflectionView, "translationY", translationY).apply {
            duration = 0
            start()
        }
    }

    private fun resetReflection() {
        ObjectAnimator.ofFloat(reflectionView, "translationY", reflectionView.height.toFloat()).apply {
            duration = 300
            start()
        }
    }
}