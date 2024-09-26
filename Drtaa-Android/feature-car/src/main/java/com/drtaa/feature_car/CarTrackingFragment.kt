package com.drtaa.feature_car

import android.view.View
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
            color = ContextCompat.getColor(requireContext(), com.drtaa.core_ui.R.color.blue_a0ba)
            outlineColor =
                ContextCompat.getColor(requireContext(), com.drtaa.core_ui.R.color.blue_a0ba_80)
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
        observeViewModelOnMap(naverMap)
        observeState()
        viewModel.getRoute()
        binding.btnCall.setOnClickListener {
            showLoading()
            if (viewModel.currentRentDetail.value?.rentStatus == "reserved") {
                viewModel.callFirstAssignedCar()
            } else {
                viewModel.callAssignedCar(STARBUCKS)
            }
        }
        carMarker.map = naverMap
        naverMap.moveCameraTo(STARBUCKS.latitude, STARBUCKS.longitude)
        binding.btnTracking.setOnClickListener {
            viewModel.toggleTrackingState()
        }

        binding.btnReturn.setOnClickListener {
            viewModel.completeRent()
        }
    }

    private fun observeState() {
        viewModel.mqttConnectionStatus.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { status ->
                Timber.tag("connect mqtt").d("$status")
                if (status == 1) {
                    dismissLoading()
                    viewModel.startPublish()
                    showSnackBar("MQTT 연결 성공")
                } else if (status == -1) {
                    Timber.tag("mqtt").d("mqtt 연결 실패")
                } else {
                    dismissLoading()
                    navigatePopBackStack()
                    showSnackBar("다시 접속해 주세요")
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun observeViewModelOnMap(naverMap: NaverMap) {
        viewModel.trackingState.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach {
            binding.btnTracking.text = if (it) {
                "차량추적 ON"
            } else {
                "차량추적 OFF"
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)

        viewModel.firstCall.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach {
            if (it) {
                showSnackBar("첫 렌트 요청 장소로 호출됩니다")
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
                    viewModel.toggleTrackingState()
                    showSnackBar("차량을 호출합니다")
                }
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)

        viewModel.routeData.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach { routeData ->
            if(routeData.isEmpty()) return@onEach
            Timber.tag("pathFrag").d("$routeData")
            pathOverlay.apply {
                coords = routeData.map { LatLng(it.lat, it.lon) }
                map = naverMap
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun pathOverlayProgress(gps: LatLng) {
        val path = viewModel.routeData.value
        val end = path.last().index
        Timber.tag("gps progress").d("$path || $end")
        runCatching {
            path.first {
                it.lat == gps.latitude && it.lon == gps.longitude
            }.index
        }.onSuccess { progress ->
            Timber.tag("gps progress").d("path and end: $path || $end -- ${(progress.toFloat() / end.toFloat()).toDouble()}")
            pathOverlay.progress = (progress.toFloat() / end.toFloat()).toDouble()
        }.onFailure {
            Timber.tag("gps progress").d("매칭 안됨")
        }
    }

    override fun iniView() {

    }

    companion object {
        private const val ICON_SIZE = 100
        private val STARBUCKS = LatLng(37.576636819990284, 126.89879021208397)
//        private val SANGAM_LATLNG = LatLng(37.57569116736151, 126.90039723462993)
    }
}