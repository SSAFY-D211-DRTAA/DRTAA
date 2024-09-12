package com.drtaa.core_ui

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import androidx.core.app.ActivityCompat
import com.gun0912.tedpermission.PermissionListener
import com.gun0912.tedpermission.normal.TedPermission
import timber.log.Timber

fun locationPermission(){
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
        TedPermission.create()
            .setDeniedMessage("위치 권한을 허용해주세요.")
            .setDeniedCloseButtonText("닫기")
            .setGotoSettingButtonText("설정")
            .setPermissions(
                android.Manifest.permission.ACCESS_FINE_LOCATION,
                android.Manifest.permission.ACCESS_COARSE_LOCATION
            )
            .setPermissionListener(object : PermissionListener {

                override fun onPermissionGranted() {
                    Timber.d("위치 권한 허용")
                }

                override fun onPermissionDenied(deniedPermissions: MutableList<String>?) {

                }
            })
            .check()
    }
}

fun checkLocationPermission(context: Context) {
    if (ActivityCompat.checkSelfPermission(
            context,
            Manifest.permission.ACCESS_FINE_LOCATION,
        ) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(
            context,
            Manifest.permission.ACCESS_COARSE_LOCATION,
        ) != PackageManager.PERMISSION_GRANTED
    ) {
        return
    }
}

fun Context.requestMapPermission(complete: () -> Unit) {
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
        TedPermission.create()
            .setDeniedMessage("No 권한")
            .setPermissions(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION,
            )
            .setPermissionListener(object : PermissionListener {
                override fun onPermissionGranted() { // 권한이 허용됐을 때
                    Timber.d("권한 허용 완료!")
                    complete()
                }

                override fun onPermissionDenied(deniedPermissions: MutableList<String>?) { // 권한이 거부됐을 때
                    Timber.d("권한 허용이 거부됨")
                }
            })
            .check()
    }
}