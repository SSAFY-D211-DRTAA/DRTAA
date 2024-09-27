package com.drtaa.feature_taxi

import android.graphics.Color
import androidx.fragment.app.viewModels
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_map.base.BaseMapFragment
import com.drtaa.feature_taxi.databinding.FragmentTaxiSummaryBinding
import com.drtaa.feature_taxi.viewmodel.TaxiSummaryViewModel
import com.drtaa.feature_taxi.viewmodel.TaxiViewModel
import com.naver.maps.geometry.LatLng
import com.naver.maps.geometry.LatLngBounds
import com.naver.maps.map.CameraUpdate
import com.naver.maps.map.MapView
import com.naver.maps.map.NaverMap
import com.naver.maps.map.overlay.ArrowheadPathOverlay
import com.naver.maps.map.overlay.PathOverlay
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.launch
import timber.log.Timber

@AndroidEntryPoint
class TaxiSummaryFragment :
    BaseMapFragment<FragmentTaxiSummaryBinding>(R.layout.fragment_taxi_summary) {

    private val taxiViewModel: TaxiViewModel by hiltNavGraphViewModels(R.id.nav_graph_taxi)
    private val taxiSummaryViewModel: TaxiSummaryViewModel by viewModels()

    override var mapView: MapView? = null

    private var pathOverlay: ArrowheadPathOverlay? = null

    private lateinit var naverMap: NaverMap

    override fun initMapView() {
        mapView = binding.mvTaxiSummary
        mapView?.getMapAsync(this)
    }

    override fun initOnMapReady(naverMap: NaverMap) {
        this.naverMap = naverMap
        naverMap.uiSettings.isLocationButtonEnabled = false

        initObserve()
        requestRoute()
    }

    override fun iniView() {
        initEvent()
    }

    private fun initObserve() {
        taxiViewModel.routeOverlay.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { routePoints ->
                routePoints?.let {
                    drawRoute(it)
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        taxiViewModel.routeInfo.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { routeInfo ->
                routeInfo?.let {
                    binding.apply {
                        tvTaxiDuration.text = "${routeInfo.totalTime/Hour}분"
                        tvTaxiPrice.text = "${routeInfo.taxiFare}원"
                    }
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun drawRoute(routePoints: List<LatLng>) {
        Timber.d("경로 그리기...중..")
        pathOverlay?.map = null
        pathOverlay = ArrowheadPathOverlay().apply {
            coords = routePoints
            map = naverMap
            width = 20
            color = Color.BLUE
            outlineWidth = 2
            outlineColor = Color.WHITE
        }

        val bounds = LatLngBounds.Builder().include(routePoints).build()
        naverMap.moveCamera(CameraUpdate.fitBounds(bounds,100))
    }

    private fun requestRoute() {
        val start = taxiViewModel.taxiStartLocation.value
        val end = taxiViewModel.taxiEndLocation.value
        if (start != null && end != null) {
            taxiViewModel.getRoute(start, end)
        }
    }

    private fun initEvent() {

    }

    companion object {
        const val Hour = 60
    }

}