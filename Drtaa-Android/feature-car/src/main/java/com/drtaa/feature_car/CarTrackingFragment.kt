package com.drtaa.feature_car

import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_map.base.BaseMapFragment
import com.drtaa.core_map.moveCameraTo
import com.drtaa.feature_car.databinding.FragmentCarTrackingBinding
import com.drtaa.feature_car.viewmodel.CarViewModel
import com.naver.maps.geometry.LatLng
import com.naver.maps.map.CameraPosition
import com.naver.maps.map.MapView
import com.naver.maps.map.NaverMap
import com.naver.maps.map.overlay.Marker
import com.naver.maps.map.overlay.OverlayImage
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach

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
        viewModel.startPublish()
        mapView = binding.mapView
        mapView?.getMapAsync(this)
    }

    override fun initOnMapReady(naverMap: NaverMap) {
        carMarker.map = naverMap
        naverMap.cameraPosition = CameraPosition(LatLng(37.5665, 126.9780), DEFAULT_ZOOM_LEVEL)
        binding.btnTrackingOn.setOnClickListener {
            viewModel.toggleTrackingState()
        }

        viewModel.gpsData.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach { gps ->
            carMarker.apply {
                position = gps
            }
            if(viewModel.trackingState.value){
                naverMap.moveCameraTo(carMarker.position.latitude, carMarker.position.longitude)
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    override fun iniView() {

    }

    override fun onDestroy() {
        super.onDestroy()
        viewModel.stopPublish()
    }

    companion object {
        private const val ICON_SIZE = 64
        private const val DEFAULT_ZOOM_LEVEL = 16.0
    }
}