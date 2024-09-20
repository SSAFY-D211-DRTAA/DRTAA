package com.drtaa.feature_home

import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_home.adapter.RentHistoryAdapter
import com.drtaa.feature_home.databinding.FragmentRentHistoryBinding
import com.drtaa.feature_home.viewmodel.RentHistoryViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach

@AndroidEntryPoint
class RentHistoryFragment :
    BaseFragment<FragmentRentHistoryBinding>(R.layout.fragment_rent_history) {

    private val rentHistoryViewModel: RentHistoryViewModel by viewModels()

    private val rentHistoryAdapter = RentHistoryAdapter()

    override fun initView() {
        initRV()
        initObserve()
    }

    private fun initObserve() {
        rentHistoryViewModel.rentHistory.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { rentHistory ->
                rentHistoryAdapter.submitList(rentHistory)
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initRV() {
        binding.rvRentHistory.adapter = rentHistoryAdapter
    }
}