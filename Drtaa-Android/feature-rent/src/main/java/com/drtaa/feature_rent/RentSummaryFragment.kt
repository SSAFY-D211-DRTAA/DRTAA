package com.drtaa.feature_rent

import android.view.View
import androidx.fragment.app.viewModels
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_rent.databinding.FragmentRentSummaryBinding
import com.drtaa.feature_rent.viewmodel.RentSummaryViewModel
import com.drtaa.feature_rent.viewmodel.RentViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach

@AndroidEntryPoint
class RentSummaryFragment :
    BaseFragment<FragmentRentSummaryBinding>(R.layout.fragment_rent_summary) {

    private val rentViewModel: RentViewModel by hiltNavGraphViewModels(R.id.nav_graph_rent)
    private val rentSummaryViewModel: RentSummaryViewModel by viewModels()

    override fun initView() {
        showLoading()

        initEvent()
        initObserve()

        rentSummaryViewModel.getUnAssignedCar()
        rentViewModel.getRentInfo()
    }

    private fun initObserve() {
        rentSummaryViewModel.assignedCar.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { result ->
                dismissLoading()
                if (result == null) {
                    binding.tvSummaryError.visibility = View.VISIBLE
                } else if (!result.available) {
                    binding.tvSummaryNoCar.visibility = View.VISIBLE
                } else {
                    binding.rentCar = result.rentCar
                    binding.svSummary.visibility = View.VISIBLE
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        rentViewModel.rentInfo.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { result ->
                if (result == null) {
                    return@onEach
                }

                binding.rentInfo = result
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initEvent() {
        binding.btnSummaryPay.setOnClickListener {
//
        }
    }
}
