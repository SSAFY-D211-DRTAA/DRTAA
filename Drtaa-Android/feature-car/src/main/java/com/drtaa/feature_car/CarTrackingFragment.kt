package com.drtaa.feature_car

import android.content.res.ColorStateList
import android.graphics.Color
import androidx.core.content.ContextCompat
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_map.base.BaseMapFragment
import com.drtaa.core_map.moveCameraTo
import com.drtaa.core_ui.showSnackBar
import com.drtaa.feature_car.databinding.FragmentCarTrackingBinding
import com.drtaa.feature_car.viewmodel.CarViewModel
import com.naver.maps.geometry.LatLng
import com.naver.maps.map.MapView
import com.naver.maps.map.NaverMap
import com.naver.maps.map.overlay.Marker
import com.naver.maps.map.overlay.OverlayImage
import com.naver.maps.map.overlay.PathOverlay
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber

@AndroidEntryPoint
class CarTrackingFragment :
    BaseMapFragment<FragmentCarTrackingBinding>(R.layout.fragment_car_tracking) {
    private val viewModel: CarViewModel by hiltNavGraphViewModels<CarViewModel>(R.id.nav_graph_car)
    override var mapView: MapView? = null

    private var carMarker: Marker = Marker().apply {
        icon = OverlayImage.fromResource(R.drawable.ic_car_marker)
        width = ICON_SIZE
        height = ICON_SIZE
        position = LatLng(0.0, 0.0)
    }

    private val pathOverlay by lazy {
        PathOverlay().apply {
            color = Color.RED
            outlineColor = Color.RED
            passedColor = ContextCompat.getColor(requireContext(), android.R.color.transparent)
            passedOutlineColor =
                ContextCompat.getColor(requireContext(), android.R.color.transparent)
            outlineWidth = 5
        }
    }

    override fun initMapView() {
        showLoading()
        mapView = binding.mapView
        mapView?.getMapAsync(this)
    }

    override fun initOnMapReady(naverMap: NaverMap) {
        observeMQTTOnMap(naverMap)
        observeCarTracking()
        carMarker.map = naverMap
        binding.btnCall.setOnClickListener {
            showLoading()
            // isInProgressed -> 재호출, isReserved -> 첫 호출
            if (viewModel.isInProgress.value) {
                // 재호출 시 스타벅스로 지정
                viewModel.recallAssignedCar(STARBUCKS)
            } else if (viewModel.isReserved.value) {
                viewModel.callFirstAssignedCar()
            } else {
                showSnackBar("배정된 차량이 없습니다")
            }
        }

        binding.btnTracking.setOnClickListener {
            viewModel.toggleTrackingState()
        }

        binding.btnReturn.setOnClickListener {
            viewModel.returnRent()
        }
    }

    private fun observeCarTracking() {
        viewModel.trackingState.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach { isTracking ->
            binding.btnTracking.backgroundTintList = if (isTracking) {
                null
            } else {
                ColorStateList.valueOf(ContextCompat.getColor(requireContext(), R.color.black))
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)

        viewModel.isReturn.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach {
            if (it) {
                showSnackBar("반납 성공")
                navigatePopBackStack()
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)

        viewModel.isFirst.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach {
            if (it) {
                dismissLoading()
                showSnackBar("첫 렌트 요청 장소로 호출합니다")
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)

        viewModel.isRecall.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach {
            if (it) {
                dismissLoading()
                showSnackBar("요청 장소로 호출하겠습니다")
            } else {
                dismissLoading()
                showSnackBar("렌트카 호출 실패!!!")
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun observeMQTTOnMap(naverMap: NaverMap) {
        viewModel.mqttConnectionStatus.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { status ->
                Timber.tag("connect mqtt").d("$status")
                if (status == 1){
                    dismissLoading()
                    viewModel.isPublising()?.let {
                        viewModel.startGPSPublish()
                    }
                    showSnackBar("MQTT 연결 성공")
                    viewModel.getRoute()
                } else if (status == -1) {
                    Timber.tag("mqtt").d("mqtt 연결 실패")
                } else {
                    dismissLoading()
                    navigatePopBackStack()
                    showSnackBar("다시 접속해 주세요")
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        viewModel.gpsData.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach { gps ->
            Timber.tag("gps").d("$gps")
            pathOverlayProgress(gps)
            carMarker.apply {
                position = gps
                if (viewModel.trackingState.value) {
                    naverMap.moveCameraTo(position.latitude, position.longitude)
                }
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)

        viewModel.carPosition.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach {
            dismissLoading()
            if (it.latitude == 0.0 && it.longitude == 0.0) {
                showSnackBar("렌트한 차량이 없습니다")
            } else {
                carMarker.apply {
                    position = LatLng(it.latitude, it.longitude)
                    naverMap.moveCameraTo(position.latitude, position.longitude)
                    if (!viewModel.trackingState.value) viewModel.toggleTrackingState()
                    showSnackBar("차량을 호출합니다")
                }
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)

        viewModel.routeData.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach { routeData ->
            if (routeData.isEmpty()) return@onEach
            Timber.tag("pathFrag").d("$routeData")
            pathOverlay.apply {
                coords = routeData.map { LatLng(it.lat, it.lon) }
                map = naverMap
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun pathOverlayProgress(gps: LatLng) {
        val path = viewModel.routeData.value
        if (path.isNotEmpty()) {
            val end = path.last().idx
            Timber.tag("gps path").d("$path || $end")

            // 현재 GPS와 경로의 점들 간의 거리를 계산하여 THRESHOLD 범위 내에서 가장 가까운 지점을 찾음
            val closestPoint = path.minByOrNull { point ->
                gps.distanceTo(LatLng(point.lat, point.lon))
            }

            // 가장 가까운 지점과의 거리가 THRESHOLD 범위 이내일 경우에만 진행 상황 계산
            closestPoint?.let { matchedPoint ->
                val distanceToClosestPoint =
                    gps.distanceTo(LatLng(matchedPoint.lat, matchedPoint.lon))
                if (distanceToClosestPoint <= THRESHOLD) {
                    val progress = matchedPoint.idx
                    Timber.tag("gps progress")
                        .d("path and end: $path || $end -- ${(progress.toFloat() / end.toFloat()).toDouble()}")
                    pathOverlay.progress = (progress.toFloat() / end.toFloat()).toDouble()
                } else {
                    Timber.tag("gps progress")
                        .d("매칭 안됨: 가장 가까운 지점이 허용 범위 밖입니다. $distanceToClosestPoint")
                }
            } ?: run {
                Timber.tag("gps progress").d("매칭 안됨: 경로가 존재하지 않음.")
            }
        }
    }

    override fun iniView() {
//
    }

    companion object {
        private const val THRESHOLD = 10.0
        private const val ICON_SIZE = 100
        private val STARBUCKS = LatLng(37.576636819990284, 126.89879021208397)
//        private val SANGAM_LATLNG = LatLng(37.57569116736151, 126.90039723462993)
    }
}
