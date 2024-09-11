package com.drtaa.core_map.base

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.annotation.IdRes
import androidx.annotation.UiThread
import androidx.databinding.DataBindingUtil
import androidx.databinding.ViewDataBinding
import androidx.fragment.app.Fragment
import androidx.navigation.NavDirections
import androidx.navigation.fragment.findNavController
import com.drtaa.core_map.LOCATION_PERMISSION_REQUEST_CODE
import com.drtaa.core_map.setup
import com.naver.maps.map.MapView
import com.naver.maps.map.NaverMap
import com.naver.maps.map.OnMapReadyCallback
import com.naver.maps.map.util.FusedLocationSource

abstract class BaseMapFragment<T : ViewDataBinding>(private val layoutResId: Int) :
    Fragment(),
    OnMapReadyCallback {

    private var _binding: T? = null
    val binding get() = _binding!!

    abstract var mapView: MapView?
    private lateinit var locationSource: FusedLocationSource

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?,
    ): View {
        _binding = DataBindingUtil.inflate(inflater, layoutResId, container, false)
        binding.lifecycleOwner = viewLifecycleOwner
        initMapView()
        mapView?.getMapAsync(this)
        locationSource = FusedLocationSource(this, LOCATION_PERMISSION_REQUEST_CODE)
        return binding.root
    }

    /**
     * mapView 바인딩해주세요
     */
    abstract fun initMapView()

    @UiThread
    override fun onMapReady(naverMap: NaverMap) {
        naverMap.setup(locationSource)
        initOnMapReady(naverMap)
    }

    /**
     * onMapReady와 동일
     */
    abstract fun initOnMapReady(naverMap: NaverMap)

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        iniView()
    }

    /**
     * Fragment onViewCreated 이후에 호출되는 메서드
     */
    abstract fun iniView()

    fun navigateDestination(@IdRes action: Int) { // Navigation 이동
        findNavController().navigate(action)
    }

    fun navigateDestination(action: NavDirections) { // Navigation 이동
        findNavController().navigate(action)
    }

    override fun onStart() {
        super.onStart()
        mapView?.onStart()
    }

    override fun onResume() {
        super.onResume()
        mapView?.onResume()
    }


    override fun onPause() {
        super.onPause()
        mapView?.onPause()
    }

    override fun onSaveInstanceState(outState: Bundle) {
        super.onSaveInstanceState(outState)
        mapView?.onSaveInstanceState(outState)
    }

    override fun onStop() {
        super.onStop()
        mapView?.onStop()
    }

    override fun onDestroyView() {
        super.onDestroyView()
        mapView?.onDestroy()
        _binding = null
    }

    override fun onLowMemory() {
        super.onLowMemory()
        mapView?.onLowMemory()
    }

}
