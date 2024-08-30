package com.drtaa.feature_main

import androidx.core.view.isVisible
import androidx.navigation.NavController
import androidx.navigation.fragment.NavHostFragment
import androidx.navigation.ui.setupWithNavController
import com.drtaa.core_ui.base.BaseActivity
import com.drtaa.feature_main.databinding.ActivityMainBinding
import com.drtaa.feature_main.util.Page
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MainActivity : BaseActivity<ActivityMainBinding>(R.layout.activity_main) {
    private lateinit var navController: NavController

    override fun init() {
        initBottomNavBar()
    }

    private fun initBottomNavBar() {
        val navHostFragment =
            supportFragmentManager.findFragmentById(R.id.nav_host) as NavHostFragment
        navController = navHostFragment.navController

        binding.apply {
            bottomNavigationBar.background = null
            bottomNavigationBar.setupWithNavController(navController)
            bottomNavigationBar.setOnItemReselectedListener {

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