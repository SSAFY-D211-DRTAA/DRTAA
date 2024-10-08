package com.drtaa.feature_home

import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.bumptech.glide.Glide
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_home.databinding.FragmentHomeBinding
import com.drtaa.feature_home.viewmodel.HomeViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber

@AndroidEntryPoint
class HomeFragment : BaseFragment<FragmentHomeBinding>(R.layout.fragment_home) {
    private val homeViewModel: HomeViewModel by viewModels()

    override fun initView() {
        Glide.with(this).load(R.raw.car_loading).into(binding.ivHomeCarDriving)
        initObserve()
        initEvent()
    }

    override fun onResume() {
        super.onResume()
        homeViewModel.refreshUserData()
        homeViewModel.getRentStatus()
    }

    private fun initObserve() {
        homeViewModel.currentUser.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { result ->
                if (result == null) return@onEach
                binding.socialUser = result
                Timber.d("지금 현재 유저는?? $result")
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        homeViewModel.rentStatus.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { rentStatus ->
                binding.tvHomeRentStatus.text = rentStatus
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initEvent() {
        binding.cvHomeRent.setOnClickListener {
            navigateDestination(R.id.action_home_to_rent)
        }

        binding.cvHomeTaxi.setOnClickListener {
            navigateDestination(R.id.action_homeFragment_to_taxiFragment)
        }

        binding.cvHomePlan.setOnClickListener {
            navigateDestination(R.id.action_homeFragment_to_nav_graph_plan)
        }
    }
}