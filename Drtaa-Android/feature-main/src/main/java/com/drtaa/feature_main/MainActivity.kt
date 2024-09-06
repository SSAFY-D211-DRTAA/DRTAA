package com.drtaa.feature_main

import android.annotation.SuppressLint
import android.view.View
import androidx.annotation.UiThread
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.coordinatorlayout.widget.CoordinatorLayout
import androidx.core.view.isVisible
import androidx.fragment.app.FragmentContainerView
import androidx.navigation.NavController
import androidx.navigation.fragment.NavHostFragment
import com.drtaa.core_ui.base.BaseActivity
import com.drtaa.feature_main.databinding.ActivityMainBinding
import com.drtaa.feature_main.util.Constant
import com.google.android.material.bottomsheet.BottomSheetBehavior
import com.naver.maps.map.MapFragment
import com.naver.maps.map.NaverMap
import com.naver.maps.map.OnMapReadyCallback
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MainActivity : BaseActivity<ActivityMainBinding>(R.layout.activity_main), OnMapReadyCallback {
    private lateinit var navController: NavController
    private lateinit var bottomSheetBehavior: BottomSheetBehavior<ConstraintLayout>
    private lateinit var mapContainer: FragmentContainerView

    override fun init() {
        initMap()
        initBottomSheet()
        initNavController()
        initBottomMenu()
    }

    @SuppressLint("CommitTransaction")
    private fun initMap() {
        val fm = supportFragmentManager
        val mapFragment = fm.findFragmentById(R.id.map) as MapFragment?
            ?: MapFragment.newInstance().also {
                fm.beginTransaction().add(R.id.map, it).commit()
            }
        mapFragment.getMapAsync(this)
    }

    @UiThread
    override fun onMapReady(naverMap: NaverMap) {
        mapContainer = binding.map
    }

    private fun initBottomSheet() {
        val height60dp = resources.getDimensionPixelSize(R.dimen.bottom_sheet_height_60dp)
        val height260dp = resources.getDimensionPixelSize(R.dimen.bottom_sheet_height_260dp)
        val height650dp = resources.getDimensionPixelSize(R.dimen.bottom_sheet_height_650dp)

        bottomSheetBehavior = BottomSheetBehavior.from(binding.bottomSheet)

        bottomSheetBehavior.apply {
            halfExpandedRatio = height260dp.toFloat() / height650dp
            expandedOffset = height60dp
            peekHeight = height260dp / 2
            state = BottomSheetBehavior.STATE_HALF_EXPANDED
            isFitToContents = false
            isDraggable = true
            isHideable = false

            addBottomSheetCallback(object : BottomSheetBehavior.BottomSheetCallback() {
                override fun onStateChanged(bottomSheet: View, newState: Int) {
                    // Nothing
                }

                override fun onSlide(bottomSheet: View, slideOffset: Float) {
                    updateMapViewPadding(bottomSheet.top)
                }
            })
        }
    }

    private fun initNavController() {
        val navHostFragment =
            supportFragmentManager.findFragmentById(R.id.nav_host) as NavHostFragment
        navController = navHostFragment.navController
        navControllerSetting()
    }

    private fun initBottomMenu() {
        binding.apply {
            menuHome.setOnClickListener {
                navigateTo(com.drtaa.feature_home.R.id.nav_graph_home)
            }
            menuTicket.setOnClickListener {
                navigateTo(com.drtaa.feature_ticket.R.id.nav_graph_map)
            }
            menuMypage.setOnClickListener {
                showMyPage()
            }
        }
    }

    private fun navigateTo(destinationId: Int) {
        binding.containterMyPage.visibility = View.GONE
        binding.bottomSheet.visibility = View.VISIBLE
        bottomSheetBehavior.state = BottomSheetBehavior.STATE_HALF_EXPANDED
        navController.navigate(destinationId)
    }

    private fun showMyPage() {
        binding.containterMyPage.visibility = View.VISIBLE
        binding.bottomSheet.visibility = View.GONE
    }

    private fun navControllerSetting() {
        navController.addOnDestinationChangedListener { _, destination, _ ->
            val page = Constant.Page.fromId(destination.id)
            binding.llBottomMenu.isVisible = page?.hideBottomNav != true
        }
    }

    private fun updateMapViewPadding(bottomSheetTop: Int) {
        val screenHeight = resources.displayMetrics.heightPixels
        val mapMargin = screenHeight - bottomSheetTop - BOTTOM_MARGIN

        (mapContainer.layoutParams as CoordinatorLayout.LayoutParams).apply {
            bottomMargin = mapMargin
            mapContainer.layoutParams = this
        }
    }

    companion object {
        private const val BOTTOM_MARGIN = 200
    }
}