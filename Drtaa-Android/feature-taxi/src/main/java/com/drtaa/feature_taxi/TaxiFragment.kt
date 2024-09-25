package com.drtaa.feature_taxi

import android.view.View
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import com.drtaa.core_map.base.BaseMapFragment
import com.drtaa.core_map.setCustomLocationButton
import com.drtaa.feature_taxi.databinding.FragmentTaxiBinding
import com.drtaa.feature_taxi.viewmodel.TaxiViewModel
import com.google.android.material.bottomsheet.BottomSheetBehavior
import com.google.android.material.bottomsheet.BottomSheetBehavior.BottomSheetCallback
import com.naver.maps.map.LocationTrackingMode
import com.naver.maps.map.MapView
import com.naver.maps.map.NaverMap
import com.naver.maps.map.util.FusedLocationSource
import dagger.hilt.android.AndroidEntryPoint
import timber.log.Timber

@AndroidEntryPoint
class TaxiFragment :
    BaseMapFragment<FragmentTaxiBinding>(R.layout.fragment_taxi) {
    private val taxiViewModel: TaxiViewModel by hiltNavGraphViewModels(R.id.nav_graph_taxi)

    override var mapView: MapView? = null

    private lateinit var locationSource: FusedLocationSource
    private lateinit var behavior: BottomSheetBehavior<ConstraintLayout>

    override fun initMapView() {
        mapView = binding.mvTaximap
    }

    override fun initOnMapReady(naverMap: NaverMap) {
        naverMap.uiSettings.isLocationButtonEnabled = false
        naverMap.setCustomLocationButton(binding.layoutRentSearchBottomSheet.ivRentSearchCurrentLocation)
        locationSource = FusedLocationSource(this, LOCATION_PERMISSION_REQUEST_CODE)
        naverMap.locationTrackingMode = LocationTrackingMode.Follow
    }

    override fun iniView() {
        initBottomSheet()
    }

    private fun initBottomSheet() {
        binding.layoutRentSearchBottomSheet.tvSearchNothing.visibility = View.GONE
        binding.layoutRentSearchBottomSheet.btnSearchSelect.visibility = View.GONE

        behavior = BottomSheetBehavior.from(binding.clTaxiSearchBottomSheet)
        behavior.state = BottomSheetBehavior.STATE_COLLAPSED
        behavior.isHideable = false

        behavior.peekHeight = BOTTOM_SHEET_PEEK_HEIGHT
        behavior.maxHeight = BOTTOM_SHEET_PEEK_HEIGHT * 2

        behavior.addBottomSheetCallback(object : BottomSheetCallback() {
            override fun onStateChanged(bottomSheet: View, newState: Int) {
                when (newState) {
                    BottomSheetBehavior.STATE_EXPANDED -> {
                        Timber.d("STATE_EXPANDED 펼침")
                    }

                    BottomSheetBehavior.STATE_COLLAPSED -> {
                        Timber.d("STATE_COLLAPSED 접음")
                    }

                    BottomSheetBehavior.STATE_DRAGGING -> {
                        Timber.d("STATE_DRAGGING 드래그")
                    }

                    BottomSheetBehavior.STATE_SETTLING -> {
                        Timber.d("STATE_SETTLING 고정")
                    }

                    BottomSheetBehavior.STATE_HIDDEN -> {
                        Timber.d("STATE_HIDDEN 숨김")
                    }
                }
            }

            override fun onSlide(bottomSheet: View, slideOffset: Float) {
            }
        })
    }

    companion object {
        const val BOTTOM_SHEET_PEEK_HEIGHT = 500
        const val MAP_BOTTOM_CONTENT_PADDING = 100
        const val LOCATION_PERMISSION_REQUEST_CODE = 1000
    }


}