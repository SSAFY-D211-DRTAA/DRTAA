package com.drtaa.feature_home

import androidx.fragment.app.viewModels
import com.drtaa.core_map.base.BaseMapFragment
import com.drtaa.feature_home.databinding.FragmentRentLocationBinding
import com.drtaa.feature_home.viewmodel.HomeViewModel
import com.naver.maps.map.MapView
import com.naver.maps.map.NaverMap
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class RentLocationFragment :
    BaseMapFragment<FragmentRentLocationBinding>(R.layout.fragment_rent_location) {
    private val viewModel: HomeViewModel by viewModels()
    override var mapView: MapView? = null

    override fun initMapView() {
        mapView = binding.mvMap
        mapView?.getMapAsync(this)
    }

    override fun initOnMapReady(naverMap: NaverMap) {

    }

    override fun iniView() {
        binding.btnRentLocationSearch.setOnClickListener {
            viewModel.getSearchList(binding.etRentLocationSearch.text.toString())
        }
    }
}