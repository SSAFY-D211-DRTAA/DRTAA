package com.drtaa.feature_main

import androidx.core.view.isVisible
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.navigation.NavController
import androidx.navigation.fragment.NavHostFragment
import androidx.navigation.ui.setupWithNavController
import com.drtaa.core_ui.base.BaseActivity
import com.drtaa.core_ui.component.LocationHelper
import com.drtaa.core_ui.showToast
import com.drtaa.feature_main.databinding.ActivityMainBinding
import com.drtaa.feature_main.util.Page
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

    override fun init() {
        initBottomNavBar()
        initLocationPermission()
    }

    private fun initLocationPermission() {
        lifecycleScope.launch {
            try {
                locationHelper.getLocationUpdates().flowWithLifecycle(lifecycle)
                    .onEach { location ->
                        Timber.tag("location")
                            .d("Lat: ${location.latitude}, Lng: ${location.longitude}")
                    }.launchIn(lifecycleScope)
            } catch (e: LocationHelper.LocationException) {
                when (e.message) {
                    "위치 서비스가 비활성화되어 있습니다." -> {
                        showToast("위치 서비스가 비활성화되어 있습니다.")
                        locationHelper.openLocationSettings(this@MainActivity)
                    }

                    else -> {
                        showToast("위치 권한이 필요합니다.")
                    }
                }
            }
        }
    }

    private fun initBottomNavBar() {
        val navHostFragment =
            supportFragmentManager.findFragmentById(R.id.nav_host) as NavHostFragment
        navController = navHostFragment.navController

        binding.apply {
            bottomNavigationBar.background = null
            bottomNavigationBar.setupWithNavController(navController)
            bottomNavigationBar.setOnItemReselectedListener {
                // 바텀메뉴 선택 시
            }
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
}