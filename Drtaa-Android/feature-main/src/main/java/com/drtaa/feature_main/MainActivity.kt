package com.drtaa.feature_main

import android.annotation.SuppressLint
import android.app.NotificationChannel
import android.app.NotificationManager
import android.content.Context
import android.os.Build
import android.os.Bundle
import androidx.activity.viewModels
import androidx.core.view.isVisible
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.navigation.NavController
import androidx.navigation.fragment.NavHostFragment
import androidx.navigation.ui.setupWithNavController
import com.drtaa.core_model.map.Search
import com.drtaa.core_ui.base.BaseActivity
import com.drtaa.core_ui.component.LocationHelper
import com.drtaa.core_ui.showToast
import com.drtaa.feature_main.databinding.ActivityMainBinding
import com.drtaa.feature_main.util.Page
import com.google.android.gms.tasks.OnCompleteListener
import com.google.firebase.messaging.FirebaseMessaging
import com.gun0912.tedpermission.PermissionListener
import com.gun0912.tedpermission.normal.TedPermission
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@AndroidEntryPoint
class MainActivity : BaseActivity<ActivityMainBinding>(R.layout.activity_main) {
    @Inject
    lateinit var locationHelper: LocationHelper
    private lateinit var navController: NavController
    private val viewModel: MainViewModel by viewModels()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        initFCMMessage()
    }

    override fun init() {
        initBottomNavBar()
        initLocationPermission()
        initFCM()
        initNotificationChannel(CHANNEL_ID, CHANNEL_NAME)
    }

    private fun initFCMMessage() {
        val searchRequest = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            intent.getParcelableExtra<Search>("recommend", Search::class.java)
        } else {
            intent.getParcelableExtra<Search>("recommend")
        }

        searchRequest?.let {
            val bundle = Bundle().apply {
                putParcelable("recommend", searchRequest)  // 검색 요청 데이터를 Bundle에 추가
            }
            Timber.tag("추천").d("추천 Request: $searchRequest")
            navController.navigate(com.drtaa.feature_plan.R.id.nav_graph_plan, bundle)
        }
    }

    private fun initFCM() {
        FirebaseMessaging.getInstance().token
            .addOnCompleteListener(
                OnCompleteListener { task ->
                    if (!task.isSuccessful) {
                        Timber.tag("fcm").d("FCM토큰 얻기 실패 ${task.exception}")
                        return@OnCompleteListener
                    }
                    val token = task.result
                    Timber.tag("fcm").d("FCM토큰 얻기 성공 $token")
                    viewModel.setFCMToken(token)
                }
            )
            .addOnFailureListener { task ->
                Timber.tag("fcm").d("FCM토큰 얻기 실패 $task")
            }

        FirebaseMessaging.getInstance().subscribeToTopic("1").addOnSuccessListener {
            Timber.tag("fcm").d("participantsMeetingDetailTable: success subscribed")
        }
    }

    private fun initLocationPermission() {
        lifecycleScope.launch {
            if (!locationHelper.isLocationEnabled()) {
                showToast(LocationHelper.GPS_NOT_ALLOWED)
                locationHelper.openLocationSettings(this@MainActivity)
            } else if (!locationHelper.isLocationPermissionGranted()) {
                showToast(LocationHelper.GPS_NEED)
                locationHelper.requestLocationPermission(this@MainActivity)
            } else {
                locationHelper.getCurrentLocation(this@MainActivity)
                locationHelper.getLocationUpdates().flowWithLifecycle(lifecycle)
                    .onEach { location ->
                        Timber.tag("location")
                            .d("Lat: ${location.latitude}, Lng: ${location.longitude}")
                    }.launchIn(lifecycleScope)
            }
        }
    }

    @SuppressLint("MissingPermission")
    private fun initNotificationChannel(id: String, name: String) {
        val notificationManager: NotificationManager =
            getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            TedPermission.create().setPermissionListener(object : PermissionListener {
                override fun onPermissionGranted() {
                    notificationManager.createNotificationChannel(
                        NotificationChannel(
                            id,
                            name,
                            NotificationManager.IMPORTANCE_HIGH
                        )
                    )
                }

                override fun onPermissionDenied(deniedPermissions: MutableList<String>?) {
                    showToast("알림 권한이 거부되었습니다")
                }
            })
                .setDeniedMessage("알림 권한을 허용해주세요")
                .setPermissions(
                    android.Manifest.permission.POST_NOTIFICATIONS,
                )
                .check()
        } else {
            notificationManager.createNotificationChannel(
                NotificationChannel(
                    id,
                    name,
                    NotificationManager.IMPORTANCE_HIGH
                )
            )
        }
    }

    private fun initBottomNavBar() {
        val navHostFragment =
            supportFragmentManager.findFragmentById(R.id.nav_host) as NavHostFragment
        navController = navHostFragment.navController

        binding.apply {
            bottomNavigationBar.background = null
            bottomNavigationBar.setupWithNavController(navController)
            setBottomNavHide()
        }
    }

    /** 바텀 네비게이션 숨기는 기능 */
    private fun setBottomNavHide() {
        navController.addOnDestinationChangedListener { _, destination, _ ->
            val page = Page.fromId(destination.id)
            binding.bottomNavigationBar.isVisible = page?.hideBottomNav != true
        }
    }

    companion object FCM {
        const val CHANNEL_NAME = "DRTAA"
        const val CHANNEL_ID = "DRTAA"
        const val NOTIFICATION_ID = 1001
    }
}