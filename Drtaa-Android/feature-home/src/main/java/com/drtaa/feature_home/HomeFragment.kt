package com.drtaa.feature_home


import androidx.fragment.app.viewModels
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_home.databinding.FragmentHomeBinding
import com.drtaa.feature_home.viewmodel.HomeViewModel
import dagger.hilt.android.AndroidEntryPoint


@AndroidEntryPoint
class HomeFragment : BaseFragment<FragmentHomeBinding>(R.layout.fragment_home) {
    private val viewModel: HomeViewModel by viewModels()

    override fun initView() {
        binding.apply {
            binding.viewModel = this@HomeFragment.viewModel
        }
    }

}