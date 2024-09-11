package com.drtaa.core_ui.component

import android.Manifest
import android.annotation.SuppressLint
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.location.Location
import android.location.LocationManager
import android.os.Looper
import android.provider.Settings
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.Granularity
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationResult
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.Priority
import com.google.android.gms.tasks.CancellationTokenSource
import dagger.hilt.android.qualifiers.ApplicationContext
import kotlinx.coroutines.channels.awaitClose
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.callbackFlow
import javax.inject.Inject
import javax.inject.Singleton
import kotlin.coroutines.resume
import kotlin.coroutines.resumeWithException
import kotlin.coroutines.suspendCoroutine

@Singleton
class LocationHelper @Inject constructor(
    @ApplicationContext private val context: Context
) {

    private val locationRequest =
        LocationRequest.Builder(Priority.PRIORITY_HIGH_ACCURACY, 10000).apply {
            setMinUpdateDistanceMeters(5.0f) // 5미터 이상 위치 변화가 생겨야 위치 업데이트 실행함
            setGranularity(Granularity.GRANULARITY_PERMISSION_LEVEL)
            setWaitForAccurateLocation(true)
        }.build()

    private val fusedLocationClient: FusedLocationProviderClient by lazy {
        LocationServices.getFusedLocationProviderClient(context)
    }

    private var lastLocation: Location? = null

    /**
     * 현재 사용자 위치 가져오기 (일회성)
     * @param activity : 권한 요청을 위한 액티비티
     */
    suspend fun getCurrentLocation(activity: AppCompatActivity): Location {
        if (!isLocationPermissionGranted()) {
            requestLocationPermission(activity)
        }
        if (!isLocationEnabled()) {
            throw LocationException(GPS_NOT_ALLOWED)
        }
        return getLocation()
    }

    /**
     * 가장 최근 기록된 사용자 위치 가져오기 (일회성)
     * @return 최근 위치 또는 null (저장된 위치 없을 때)
     */
    @SuppressLint("MissingPermission")
    suspend fun getLastLocation(): Location? {
        if (!isLocationPermissionGranted()) {
            throw LocationException("위치 권한이 없습니다.")
        }

        return suspendCoroutine { continuation ->
            fusedLocationClient.lastLocation
                .addOnSuccessListener { location ->
                    if (location != null) {
                        lastLocation = location
                        continuation.resume(location)
                    } else {
                        continuation.resume(lastLocation)
                    }
                }
                .addOnFailureListener { exception ->
                    continuation.resumeWithException(exception)
                }
        }
    }


    private fun isLocationPermissionGranted(): Boolean {
        return ContextCompat.checkSelfPermission(
            context,
            Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED
    }

    private suspend fun requestLocationPermission(activity: AppCompatActivity) =
        suspendCoroutine { continuation ->
            val requestPermissionLauncher = activity.registerForActivityResult(
                ActivityResultContracts.RequestPermission()
            ) { isGranted: Boolean ->
                if (isGranted) {
                    continuation.resume(Unit)
                } else {
                    continuation.resumeWithException(LocationException("위치 권한이 거부되었습니다."))
                }
            }
            requestPermissionLauncher.launch(Manifest.permission.ACCESS_FINE_LOCATION)
        }

    private fun isLocationEnabled(): Boolean {
        val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager
        return locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER) ||
                locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)
    }

    @SuppressLint("MissingPermission")
    private suspend fun getLocation(): Location = suspendCoroutine { continuation ->
        val cancellationTokenSource = CancellationTokenSource()
        fusedLocationClient.getCurrentLocation(
            Priority.PRIORITY_HIGH_ACCURACY,
            cancellationTokenSource.token
        ).addOnSuccessListener { location ->
            if (location != null) {
                lastLocation = location
                continuation.resume(location)
            } else {
                continuation.resumeWithException(LocationException("위치를 가져올 수 없습니다."))
            }
        }.addOnFailureListener { exception ->
            continuation.resumeWithException(exception)
        }
    }

    /**
     *  주기적으로 사용자 위치 가져오기 (10초마다)
     */
    @SuppressLint("MissingPermission")
    fun getLocationUpdates(): Flow<Location> = callbackFlow {
        if (!isLocationPermissionGranted()) {
            throw LocationException("위치 권한이 없습니다.")
        }

        if (!isLocationEnabled()) {
            throw LocationException(GPS_NOT_ALLOWED)
        }

        val callback = object : LocationCallback() {
            override fun onLocationResult(result: LocationResult) {
                result.lastLocation?.let { location ->
                    lastLocation = location
                    trySend(location).isSuccess
                }
            }
        }

        fusedLocationClient.requestLocationUpdates(
            locationRequest,
            callback,
            Looper.getMainLooper()
        ).addOnFailureListener { e ->
            close(e)
        }

        awaitClose {
            // flow 닫힐 때 Location 업데이트 멈춰줌
            fusedLocationClient.removeLocationUpdates(callback)
        }
    }

    fun openLocationSettings(activity: AppCompatActivity) {
        val intent = Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS)
        activity.startActivity(intent)
    }

    class LocationException(message: String) : Exception(message)

    companion object {
        const val GPS_NOT_ALLOWED = "위치 서비스가 비활성화되어 있습니다."
    }
}