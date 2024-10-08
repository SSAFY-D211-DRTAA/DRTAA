package com.drtaa.core_map

import android.annotation.SuppressLint
import android.view.MotionEvent
import android.view.View
import com.google.android.material.snackbar.Snackbar
import com.naver.maps.geometry.LatLng
import com.naver.maps.geometry.LatLngBounds
import com.naver.maps.map.CameraAnimation
import com.naver.maps.map.CameraUpdate
import com.naver.maps.map.LocationTrackingMode
import com.naver.maps.map.MapView
import com.naver.maps.map.NaverMap
import com.naver.maps.map.util.FusedLocationSource

const val LOCATION_PERMISSION_REQUEST_CODE = 1000
const val MIN_ZOOM = 8.0
const val MAX_ZOOM = 18.0

/**
 * 지도 스크롤이 부모 뷰의 스크롤에 영향 주지 않게 하는 메서드
 */
@SuppressLint("ClickableViewAccessibility")
fun MapView.ignoreParentScroll() {
    this.setOnTouchListener { v, event ->
        when (event.action) {
            MotionEvent.ACTION_DOWN, MotionEvent.ACTION_MOVE -> {
                this.parent.requestDisallowInterceptTouchEvent(true)
            }

            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                this.parent.requestDisallowInterceptTouchEvent(false)
            }
        }
        false
    }
}

/**
 * fusedLocationSource는 네이버 꺼
 */
fun NaverMap.setup(fusedLocationSource: FusedLocationSource) {
    locationSource = fusedLocationSource
    minZoom = MIN_ZOOM // 최소 줌 레벨
    maxZoom = MAX_ZOOM // 최대 줌 레벨
    uiSettings.isZoomControlEnabled = false // Zoom 컨트롤러 사용유무
    uiSettings.isLocationButtonEnabled = true // 현재위치 사용유무
}

/**
 * 커스텀 현재 위치 버튼 지정
 */
fun NaverMap.setCustomLocationButton(view: View) {
    view.setOnClickListener {
        this.locationTrackingMode = LocationTrackingMode.Follow
        val locationOverlay = this.locationOverlay
        if (locationOverlay.isVisible) {
            // 현재 위치로 이동
            val currentLocation = locationOverlay.position
            val cameraUpdate = CameraUpdate.scrollTo(currentLocation)
                .animate(CameraAnimation.Easing) // 부드럽게 애니메이션으로 이동
            this.moveCamera(cameraUpdate)
        } else {
            Snackbar.make(view, "현재 위치를 확인할 수 없습니다.", Snackbar.LENGTH_SHORT).show()
        }
    }
}

/**
 *  지정한 좌표로 카메라 이동
 *  @param latitude
 *  @param longitude
 */
fun NaverMap.moveCameraTo(latitude: Double, longitude: Double) {
    val cameraUpdate = CameraUpdate.scrollTo(LatLng(latitude, longitude)).animate(CameraAnimation.Easing)
    this.moveCamera(cameraUpdate)
}

/**
 *  마커를 한눈에 볼 수 있게 카메라 이동
 *  @param LatLngBounds
 */
fun NaverMap.moveCameraBounds(bounds: LatLngBounds) {
    val cameraUpdate = CameraUpdate.fitBounds(bounds, BOUNDS_PADDING).animate(CameraAnimation.Easing)
    this.moveCamera(cameraUpdate)
}

const val BOUNDS_PADDING = 70