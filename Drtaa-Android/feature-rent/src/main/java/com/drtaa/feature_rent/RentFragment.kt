package com.drtaa.feature_rent

import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_rent.databinding.FragmentRentBinding
import com.drtaa.feature_rent.viewmodel.RentViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach

@AndroidEntryPoint
class RentFragment : BaseFragment<FragmentRentBinding>(R.layout.fragment_rent) {

    private val rentViewModel: RentViewModel by hiltNavGraphViewModels(R.id.nav_graph_rent)

    override fun initView() {
        initEvent()
        initObserve()
    }

    private fun initObserve() {
        rentViewModel.rentStartLocation.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { rentStartLocation ->
                if (rentStartLocation != null) {
                    binding.tvRentStartLocation.text = rentStartLocation.title
                } else {
                    binding.tvRentStartLocation.hint = "강남역"
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        rentViewModel.rentStartDate.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { rentStartDate ->
                if (rentStartDate != null) {
                    binding.tvRentStartSchedule.text = rentStartDate
                } else {
                    binding.tvRentStartSchedule.hint = "09.09 (월) 20:00"
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        rentViewModel.rentEndDate.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { rentEndDate ->
                if (rentEndDate != null) {
                    binding.tvRentEndSchedule.text = rentEndDate
                } else {
                    binding.tvRentEndSchedule.hint = "09.09 (월) 20:00"
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initEvent() {
        binding.clRentStartLocation.setOnClickListener {
            navigateDestination(R.id.action_rentFragment_to_rentLocationFragment)
        }

        binding.clRentSchedule.setOnClickListener {
            CalendarBottomSheetDialogFragment().show(
                requireActivity().supportFragmentManager,
                "CalendarBottomSheetDialogFragment"
            )
        }
    }
}
