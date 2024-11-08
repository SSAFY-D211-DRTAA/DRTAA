package com.drtaa.core_map.base

import android.graphics.Typeface
import android.os.Bundle
import android.text.Spannable
import android.text.SpannableString
import android.text.style.StyleSpan
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
import com.naver.maps.map.overlay.InfoWindow
import com.naver.maps.map.overlay.Marker
import com.naver.maps.map.overlay.OverlayImage
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
        marker.apply {
            icon = OverlayImage.fromResource(com.drtaa.core_ui.R.drawable.ic_center_marker)
            position = LatLng(lat, lng)
            map = this@setMarker
            width = ICON_SIZE
            height = ICON_SIZE
        }
    }

    /**
     * 마커 리스트 추가
     */
    fun NaverMap.addMarker(lat: Double, lng: Double, index: Int, caption: String) {
        val marker = Marker().apply {
            position = LatLng(lat, lng)
            icon = OverlayImage.fromResource(com.drtaa.core_ui.R.drawable.ic_center_marker)
            map = this@addMarker
            captionText = caption
            width = ICON_SIZE
            height = ICON_SIZE
        }

        InfoWindow().apply {
            adapter = object : InfoWindow.DefaultTextAdapter(requireActivity()) {
                override fun getText(infoWindow: InfoWindow): CharSequence {
                    val boldText = SpannableString("${index}번째 일정").apply {
                        setSpan(
                            StyleSpan(Typeface.BOLD), 0, length,
                            Spannable.SPAN_EXCLUSIVE_EXCLUSIVE
                        )
                    }
                    return boldText
                }
            }
            open(marker)
        }

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

    companion object {
        const val ICON_SIZE = 100
    }
}
