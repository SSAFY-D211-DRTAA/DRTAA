package com.drtaa.feature_car

import android.animation.ObjectAnimator
import android.annotation.SuppressLint
import android.graphics.drawable.GradientDrawable
import android.view.MotionEvent
import android.view.View
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

    private val carViewModel: CarViewModel by hiltNavGraphViewModels<CarViewModel>(R.id.nav_graph_car)

    private lateinit var cardView: View
    private lateinit var overlayView: View
    private lateinit var reflectionView: View
    private var isTouching: Boolean = false
    private var touchStartTime: Long = 0

    override fun initView() {
        initUI()
        initObserve()
        setupCardTouchListener()
    }

    private fun initUI() {
        binding.apply {
            cardView = cvTourCard
            overlayView = viewTourOverlay
            reflectionView = viewTourReflection

            btnTrackingCar.setOnClickListener {
                navigateDestination(R.id.action_carFragment_to_carTrackingFragment)
            }

            clCarBottomTextGotoUse.setOnClickListener {
                navigateDestination(R.id.action_carFragment_to_carTrackingFragment)
            }
        }
    }

    private fun initObserve() {
        carviewModel.latestReservedId.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach {
            binding.tvReservedState.text = if (it == 0L) {
                "현재 예약된 차량이 없습니다"
            } else {
                "예약한 차량 호출하기"
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)
        carviewModel.currentRentDetail.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { currentRentDetail ->
                Timber.tag("car").d("$currentRentDetail")
                binding.apply {
                    if (currentRentDetail != null) {
                        imgCarCarimage.visibility = View.VISIBLE
                    } else {
                        clCarBottomTextGotoUse.visibility = View.VISIBLE
                        btnTrackingCar.isClickable = false
                        clCarBottomText.visibility = View.GONE
                        animeCarNorent.visibility = View.VISIBLE
                        tvTourRemainTime.text = "현재 이용중인 차량이 없습니다.."
                    }
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        carViewModel.isSuccessComplete.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isSuccess ->
                if (isSuccess) {
                    showSnackBar("반납 성공")
                } else {
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
                    // 부모 ScrollView가 터치 이벤트를 가로채지 않도록 설정
                    v.parent.requestDisallowInterceptTouchEvent(true)
                    true
                }

                MotionEvent.ACTION_MOVE -> handleActionMove(v, event) // 이동했을 때

                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    isTouching = false
                    cardView.animate()
                        .rotationX(0f)
                        .rotationY(0f)
                        .setDuration(DURATION)
                        .start()

                    resetOverlay()
                    resetReflection()
                    // 부모 ScrollView가 다시 터치 이벤트를 가로챌 수 있도록 원복
                    v.parent.requestDisallowInterceptTouchEvent(false)
                    true
                }

                else -> {
                    false
                }
            }
        }
    }

    private fun isLongTouch() = System.currentTimeMillis() - touchStartTime > TOUCH_PRESS_TIME

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
        var rotateX = (centerY - y) / centerY * MAX_ROTATION
        var rotateY = (x - centerX) / centerX * MAX_ROTATION

        rotateX = rotateX.coerceIn(-MAX_ROTATION, MAX_ROTATION)
        rotateY = rotateY.coerceIn(-MAX_ROTATION, MAX_ROTATION)

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
        const val DURATION = 300L
        const val TOUCH_PRESS_TIME = 100
        const val MAX_ROTATION = 15f
    }
}