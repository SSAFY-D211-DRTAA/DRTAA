package com.drtaa.feature_home

import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_home.databinding.FragmentRentBinding
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class RentFragment : BaseFragment<FragmentRentBinding>(R.layout.fragment_rent) {

    override fun initView() {
        initEvent()
    }

    private fun initEvent() {
        binding.btnHomeCarRent.setOnClickListener {
            navigateDestination(R.id.action_rentFragment_to_rentLocationFragment)
        }
    }
}