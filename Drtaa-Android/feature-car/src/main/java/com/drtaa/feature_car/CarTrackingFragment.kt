package com.drtaa.feature_car

import android.view.View
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

    override fun initMapView() {
        showLoading()
        mapView = binding.mapView
        mapView?.getMapAsync(this)
    }

    override fun initOnMapReady(naverMap: NaverMap) {
        observeViewModelOnMap(naverMap)
        observeState()
        binding.btnCall.setOnClickListener {
            showLoading()
            if (viewModel.currentRentDetail.value == null) {
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
                    showSnackBar("MQTT 연결 성공")
                } else if (status == -1) {
                    Timber.tag("mqtt").d("mqtt 연결 실패")
                }else{
                    dismissLoading()
                    navigatePopBackStack()
                    showSnackBar("다시 접속해 주세요")
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun observeViewModelOnMap(naverMap: NaverMap) {
        viewModel.trackingState.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach {
            binding.btnTracking.text = if (it) {
                viewModel.startPublish()
                "차량추적 ON"
            } else {
                viewModel.stopPublish()
                "차량추적 OFF"
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)

        viewModel.firstCall.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach {
            if (it) {
                showSnackBar("첫 렌트 요청 장소로 호출됩니다")
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)

        viewModel.gpsData.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach { gps ->
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
                    binding.btnCall.visibility = View.GONE
                    binding.btnTracking.visibility = View.VISIBLE
                    showSnackBar("차량을 호출합니다")
                }
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    override fun iniView() {
    }

    override fun onResume() {
        super.onResume()
        viewModel.initMQTT()
    }

    override fun onDestroy() {
        Timber.tag("onDestroy").d("onDestroy")
        super.onDestroy()
        viewModel.disconnectMQTT()
        viewModel.stopPublish()
        viewModel.clearMqttStatus()
    }

    companion object {
        private const val ICON_SIZE = 100
        private val STARBUCKS = LatLng(37.576636819990284, 126.89879021208397)
//        private val SANGAM_LATLNG = LatLng(37.57569116736151, 126.90039723462993)
    }
}