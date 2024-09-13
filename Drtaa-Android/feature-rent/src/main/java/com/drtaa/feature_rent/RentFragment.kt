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
    }

    private fun initEvent() {
        binding.clRentStartLocation.setOnClickListener {
            navigateDestination(R.id.action_rentFragment_to_rentLocationFragment)
        }
    }
}
