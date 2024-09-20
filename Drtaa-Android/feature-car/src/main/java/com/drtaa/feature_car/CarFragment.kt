package com.drtaa.feature_car

import android.animation.ObjectAnimator
import android.annotation.SuppressLint
import android.graphics.drawable.GradientDrawable
import android.view.MotionEvent
import android.view.View
import android.widget.ImageView
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.core_ui.showSnackBar
import com.drtaa.feature_car.databinding.FragmentCarBinding
import com.drtaa.feature_car.viewmodel.CarViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber

@AndroidEntryPoint
class CarFragment : BaseFragment<FragmentCarBinding>(R.layout.fragment_car) {

    private val viewModel: CarViewModel by hiltNavGraphViewModels<CarViewModel>(R.id.nav_graph_car)

    private lateinit var cardView: View
    private lateinit var overlayView: View
    private lateinit var reflectionView: View
    private lateinit var cardImage: ImageView
    private var isTouching: Boolean = false
    private var isNeko: Boolean = true
    private var touchStartTime: Long = 0

    override fun initView() {
        binding.apply {
            cardView = cvTourCard
            overlayView = viewTourOverlay
            reflectionView = viewTourReflection
            cardImage = ivTourCard

            btnMqtt.setOnClickListener {
                viewModel.startPublish()
            }

            btnTrackingCar.setOnClickListener {
                navigateDestination(R.id.action_carFragment_to_carTrackingFragment)
            }

            btnReturn.setOnClickListener {
                viewModel.completeRent()
            }
        }
        setupCardTouchListener()
        initObserve()
    }

    private fun initObserve() {
        viewModel.currentRentDetail.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { currentRentDetail ->
                if (currentRentDetail != null) {
                    binding.clCurrentRent.visibility = View.VISIBLE
                } else {
                    binding.tvNoCurrentRent.visibility = View.VISIBLE
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        viewModel.isSuccessComplete.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isSuccess ->
                if (isSuccess) {
                    Timber.d("반납 성공")
                    showSnackBar("반납 성공")
                } else {
                    Timber.d("반납 실패")
                    showSnackBar("반납 실패")
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
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

                MotionEvent.ACTION_MOVE -> handleActionMove(v, event) // 이동했을 때

                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    if (isTouching && isShortTouch()) {
                        isNeko = !isNeko // true false 전환
                        val newImageResource =
                            if (isNeko) R.drawable.kanna else R.drawable.kanna // 이미지 전환, 나중에는 view 전환으로 하면 될 듯?
                        cardImage.setImageResource(newImageResource) // 칸나칸나 이미지 적용
                    }

                    isTouching = false
                    cardView.animate()
                        .rotationX(0f)
                        .rotationY(0f)
                        .setDuration(DURATION)
                        .start()

                    resetOverlay()
                    resetReflection()
                    true
                }

                else -> {
                    false
                }
            }
        }
    }

    private fun isLongTouch() = System.currentTimeMillis() - touchStartTime > TOUCH_PRESS_TIME
    private fun isShortTouch() = System.currentTimeMillis() - touchStartTime <= TOUCH_PRESS_TIME

    private fun handleActionMove(v: View, event: MotionEvent): Boolean {
        if (isTouching && isLongTouch()) {
            val (x, y) = event.x to event.y
            rotateCard(v, x, y)
            updateOverlayAndReflection(v, x, y)
        }
        return true
    }

    private fun rotateCard(v: View, x: Float, y: Float) {
        val (centerX, centerY) = v.width / HALF to v.height / HALF
        val rotateX = (centerY - y) / centerY * PER
        val rotateY = (x - centerX) / centerX * PER

        cardView.animate()
            .rotationX(rotateX)
            .rotationY(rotateY)
            .setDuration(0)
            .start()
    }

    private fun updateOverlayAndReflection(v: View, x: Float, y: Float) {
        updateOverlay(x / v.width, y / v.height)
        updateReflection(y / v.height)
    }

    private fun updateOverlay(percentX: Float, percentY: Float) {
        val gradient = overlayView.background as GradientDrawable
        gradient.setGradientCenter(percentX, percentY)
        overlayView.alpha = OVERLAY_VIEW
    }

    private fun resetOverlay() {
        overlayView.animate()
            .alpha(0f)
            .setDuration(DURATION)
            .start()
    }

    private fun updateReflection(percentY: Float) {
        val translationY = (percentY - POINT_XY) * 2 * reflectionView.height
        ObjectAnimator.ofFloat(reflectionView, "translationY", translationY).apply {
            duration = 0
            start()
        }
    }

    private fun resetReflection() {
        ObjectAnimator.ofFloat(reflectionView, "translationY", reflectionView.height.toFloat())
            .apply {
                duration = DURATION
                start()
            }
    }

    companion object {
        const val OVERLAY_VIEW = 0.6f
        const val POINT_XY = 0.5f
        const val HALF = 2f
        const val PER = 10
        const val DURATION = 300L
        const val TOUCH_PRESS_TIME = 100
    }
}