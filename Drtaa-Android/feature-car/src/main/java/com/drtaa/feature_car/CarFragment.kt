package com.drtaa.feature_car

import android.Manifest
import android.animation.ObjectAnimator
import android.annotation.SuppressLint
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.drawable.GradientDrawable
import android.view.MotionEvent
import android.view.View
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_model.rent.RentDetail
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.core_ui.fitCenter
import com.drtaa.core_ui.parseLocalDateTime
import com.drtaa.core_ui.showSnackBar
import com.drtaa.feature_car.databinding.FragmentCarBinding
import com.drtaa.feature_car.viewmodel.CarStatus
import com.drtaa.feature_car.viewmodel.CarViewModel
import com.google.zxing.integration.android.IntentIntegrator
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
        showLoading()
        initUI()
        observeViewModel()
        observeStatus()
        setupCardTouchListener()
    }

    private fun initUI() {
        carViewModel.getLatestRent()
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
            btnTourQrcode.setOnClickListener {
                if (checkCameraPermission()) {
                    startQRCodeScanner()
                } else {
                    requestCameraPermission()
                }
            }
            btnGetOffQrcode.setOnClickListener {
                carViewModel.getOffCar(rentId = carViewModel.latestReservedId.value)
            }
        }
    }

    private fun toggleCarOption(input: Boolean) {
        if (!input) {
            binding.btnTourQrcode.visibility = View.GONE
            binding.btnTourExtend.visibility = View.GONE
        } else {
            binding.btnTourQrcode.visibility = View.VISIBLE
            binding.btnTourExtend.visibility = View.VISIBLE
        }
    }

    private fun observeViewModel() {
        carViewModel.latestReservedId.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { latestId ->
                when {
                    latestId == -1L -> {
                        // 예약한 차량이 없을 때
                        binding.clCarBottomTextGotoUse.isClickable = false
                        toggleCarOption(false)
                        dismissLoading()
                        binding.tvReservedState.text = "예약한 차량이 없습니다"
                    }

                    latestId > 0 -> {
                        // 예약한 차량이 있을 때
                        binding.clCarBottomTextGotoUse.isClickable = true
                        toggleCarOption(true)
                        showLoading() // 로딩 시작
                        binding.tvReservedState.text = "불러오는 중.."
                        carViewModel.getCurrentRent()
                        // currentRentDetail의 Observer에서 UI 업데이트를 처리합니다.
                    }

                    else -> {
                        binding.clCarBottomTextGotoUse.isClickable = false
                        binding.tvReservedState.text = "불러오는 중.."
                    }
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        carViewModel.currentRentDetail.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { currentRentDetail ->
                dismissLoading() // 로딩 종료
                if (currentRentDetail != null) {
                    updateCarStateUi(currentRentDetail)
                } else {
                    // 현재 이용중인 차량이 없을 때의 UI 처리
                    binding.apply {
                        clCarBottomTextGotoUse.visibility = View.VISIBLE
                        btnTrackingCar.isClickable = false
                        clCarBottomText.visibility = View.GONE
                        animeCarNorent.visibility = View.VISIBLE
                        tvTourRemainTime.text = "현재 이용중인 차량이 없습니다."
                    }
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun updateCarStateUi(currentRentDetail: RentDetail) {
        binding.apply {
            when (currentRentDetail.rentStatus) {
                "in_progress" -> {
                    tvReservedState.visibility = View.GONE
                    clCarBottomText.visibility = View.VISIBLE
                    animeCarNorent.visibility = View.GONE
                    btnTrackingCar.isClickable = true
                    tvTourRemainTime.text =
                        "남은시간 : ${currentRentDetail.rentTime * MIN} 분"
                    currentRentDetail.rentCarImg?.let {
                        imgCarCarimage.fitCenter(
                            it,
                            requireContext()
                        )
                    }
                    tvTourCarnumber.text = currentRentDetail.rentCarNumber
                    tvTourCarname.text =
                        "${currentRentDetail.rentCarManufacturer} ${currentRentDetail.rentCarModel}"
                    tvTourRentend.text =
                        currentRentDetail.rentEndTime.parseLocalDateTime()
                    tvTourRentstart.text =
                        currentRentDetail.rentStartTime.parseLocalDateTime()
                    imgCarCarimage.visibility = View.VISIBLE
                }

                "reserved" -> {
                    tvReservedState.visibility = View.VISIBLE
                    tvReservedState.text = "예약한 차량 호출하기"
                }
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        carViewModel.stopPublish()
    }

    private fun observeStatus() {
        carViewModel.drivingStatus.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { drivingStatus ->
                when (drivingStatus) {
                    CarStatus.DRIVING -> {
                        binding.btnTourQrcode.visibility = View.GONE
                        binding.btnGetOffQrcode.visibility = View.VISIBLE
                    }

                    CarStatus.PARKING -> {
                        binding.btnTourQrcode.visibility = View.VISIBLE
                        binding.btnGetOffQrcode.visibility = View.GONE
                    }

                    CarStatus.IDLE -> {
                        binding.btnTourQrcode.visibility = View.VISIBLE
                        binding.btnGetOffQrcode.visibility = View.GONE
                    }
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        carViewModel.rentState.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach { isOnCar ->
            if (isOnCar) {
                carViewModel.getCurrentRent()
            }
            Timber.tag("rentState").d("$isOnCar")
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

    private fun checkCameraPermission(): Boolean {
        return ContextCompat.checkSelfPermission(
            requireContext(),
            Manifest.permission.CAMERA
        ) == PackageManager.PERMISSION_GRANTED
    }

    private fun requestCameraPermission() {
        ActivityCompat.shouldShowRequestPermissionRationale(
            requireActivity(),
            Manifest.permission.CAMERA
        )
        requestPermissionLauncher.launch(Manifest.permission.CAMERA)
    }

    private val requestPermissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { isGranted: Boolean ->
            if (isGranted) {
                startQRCodeScanner()
            } else {
                Timber.d("카메라 권한 없음")
            }
        }

    private fun startQRCodeScanner() {
        val integrator = IntentIntegrator.forSupportFragment(this)

        integrator.apply {
            setDesiredBarcodeFormats(IntentIntegrator.QR_CODE)
            setPrompt("QR 코드를 스캔하세요")
            setCameraId(0)
            setBeepEnabled(false)
            initiateScan()
        }
    }

    override fun onActivityResult(
        requestCode: Int,
        resultCode: Int,
        data: Intent?,
    ) {
        val result = IntentIntegrator.parseActivityResult(requestCode, resultCode, data)
        if (result != null) {
            if (result.contents == null) {
                // QR 코드 스캔이 취소됨
            } else {
                // QR 코드 스캔 성공
                val scannedContent = result.contents
                setQRCode(scannedContent)
            }
        } else {
            super.onActivityResult(requestCode, resultCode, data)
        }
    }

    private fun setQRCode(qrCode: String) {
        val parts = qrCode.split(" ")
        if (parts.size == 2) {
            val qrCarId = parts[0].trim().toInt()
            val qrRentId = parts[1].trim().toLong()

            if (qrRentId != null) {
                showSnackBar("차량 ID: $qrCarId, 렌트 ID: $qrRentId")
                Timber.tag("qr").d("차량 ID: $qrCarId, 렌트 ID: $qrRentId")
                checkRentReservation(qrRentId, qrCarId)
            } else {
                showSnackBar("잘못된 QR 코드 형식입니다.")
            }
        } else {
            showSnackBar("잘못된 QR 코드 형식입니다.")
        }
    }

    private fun checkRentReservation(qrRentId: Long, qrCarId: Int) {
        Timber.tag("qr").d("qrRentId: $qrRentId, qrCarId: $qrCarId")
        val currentRentDetail = carViewModel.currentRentDetail.value
        Timber.tag("qr").d("$currentRentDetail")
        if (currentRentDetail != null) {
//            if (currentRentDetail.rentId == qrRentId && currentRentDetail.rentCarId == qrCarId) {
//                carViewModel.getOnCar(rentId = currentRentDetail.rentId!!)
//            }
            if (true) {
                carViewModel.getOnCar(rentId = currentRentDetail.rentId!!)
            } else {
                showSnackBar("배정된 차량이 아닙니다!")
            }
        } else {
            showSnackBar("배정된 차량이 없습니다!")
        }
    }

    companion object {
        const val OVERLAY_VIEW = 0.6f
        const val POINT_XY = 0.5f
        const val HALF = 2f
        const val DURATION = 300L
        const val TOUCH_PRESS_TIME = 100
        const val MAX_ROTATION = 15f
        const val MIN = 60
    }
}