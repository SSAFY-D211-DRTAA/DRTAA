package com.drtaa.feature_home

import androidx.fragment.app.viewModels
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_home.databinding.FragmentRentLocationBinding
import com.drtaa.feature_home.viewmodel.HomeViewModel
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class RentLocationFragment : BaseFragment<FragmentRentLocationBinding>(R.layout.fragment_rent_location) {
    private val viewModel: HomeViewModel by viewModels()

    override fun initView() {
        binding.btnRentLocationSearch.setOnClickListener {
            viewModel.getSearchList(binding.etRentLocationSearch.text.toString())
        }
    }
}