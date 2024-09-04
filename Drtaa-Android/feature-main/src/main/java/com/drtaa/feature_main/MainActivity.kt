package com.drtaa.feature_main

import android.view.View
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.core.view.isVisible
import androidx.navigation.NavController
import androidx.navigation.findNavController
import androidx.navigation.fragment.NavHostFragment
import com.drtaa.core_ui.base.BaseActivity
import com.drtaa.feature_main.databinding.ActivityMainBinding
import com.drtaa.feature_main.util.Page
import com.google.android.material.bottomsheet.BottomSheetBehavior
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MainActivity : BaseActivity<ActivityMainBinding>(R.layout.activity_main) {
    private lateinit var navController: NavController

    override fun init() {
        initNavController()
        initBottomMenu()
    }

    private fun initBottomSheet() {
        val bottomSheet = findViewById<ConstraintLayout>(R.id.bottom_sheet)
        val bottomSheetBehavior = BottomSheetBehavior.from(bottomSheet)
        bottomSheetBehavior.apply {
            isFitToContents = true
            isDraggable = true
            isHideable = false
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
            menuHome.setOnClickListener { navigateTo(com.drtaa.feature_home.R.id.nav_graph_home) }
            menuTicket.setOnClickListener { navigateTo(com.drtaa.feature_map.R.id.nav_graph_map) }
            menuMypage.setOnClickListener { navigateTo(com.drtaa.feature_mypage.R.id.nav_graph_my_page) }
        }
    }

    private fun navigateTo(destinationId: Int) {
        navController.navigate(destinationId)
    }

    private fun navControllerSetting() {
        navController.addOnDestinationChangedListener { _, destination, _ ->
            val page = Page.fromId(destination.id)
            binding.llBottomMenu.isVisible = page?.hideBottomNav != true
        }
    }
}