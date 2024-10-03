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
import com.drtaa.core_map.moveCameraBounds
import com.drtaa.core_map.setup
import com.drtaa.core_ui.component.LoadingDialog
import com.naver.maps.geometry.LatLng
import com.naver.maps.geometry.LatLngBounds
import com.naver.maps.map.MapView
import com.naver.maps.map.NaverMap
import com.naver.maps.map.OnMapReadyCallback
import com.naver.maps.map.overlay.Marker
import com.naver.maps.map.util.FusedLocationSource

abstract class BaseMapFragment<T : ViewDataBinding>(private val layoutResId: Int) :
    Fragment(),
    OnMapReadyCallback {

    private var _binding: T? = null
    val binding get() = _binding!!

    private val loading by lazy {
        LoadingDialog(requireActivity())
    }

    fun dismissLoading() {
        loading.dismiss()
    }

    fun showLoading() {
        loading.show()
    }

    abstract var mapView: MapView?
    private lateinit var locationSource: FusedLocationSource

    private val _markerList = mutableListOf<Marker>()
    val markerList get() = _markerList

    private val _marker = Marker()
    val marker get() = _marker

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?,
    ): View {
        _binding = DataBindingUtil.inflate(inflater, layoutResId, container, false)
        binding.lifecycleOwner = viewLifecycleOwner
        initMapView()
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

    /**
     * 단일 마커 설정
     */
    fun NaverMap.setMarker(lat: Double, lng: Double) {
        marker.map = null
        marker.position = LatLng(lat, lng)
        marker.map = this
    }

    /**
     * 마커 리스트 추가
     */
    fun NaverMap.addMarker(lat: Double, lng: Double) {
        val marker = Marker()
        marker.position = LatLng(lat, lng)
        marker.map = this

        _markerList.add(marker)
    }

    /**
     * 단일 마커 삭제
     */
    fun NaverMap.clearMarker() {
        marker.map = null
    }

    /**
     * 마커 리스트 삭제
     */
    fun NaverMap.clearMarkerList() {
        _markerList.forEach { marker ->
            marker.map = null
        }
        _markerList.clear()
    }

    fun NaverMap.adjustCamera() {
        if (_markerList.isEmpty()) return
        val boundsBuilder = LatLngBounds.Builder()
        _markerList.forEach { marker ->
            boundsBuilder.include(marker.position)
        }

        val bounds = boundsBuilder.build()
        moveCameraBounds(bounds)
    }

    fun navigateDestination(@IdRes action: Int) { // Navigation 이동
        findNavController().navigate(action)
    }

    fun navigateDestination(action: NavDirections) { // Navigation 이동
        findNavController().navigate(action)
    }

    fun navigatePopBackStack() { // 뒤로 가기
        findNavController().popBackStack()
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
