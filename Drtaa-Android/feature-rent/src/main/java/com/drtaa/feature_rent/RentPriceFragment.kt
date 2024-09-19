package com.drtaa.feature_rent

import android.graphics.drawable.Drawable
import androidx.appcompat.content.res.AppCompatResources.getDrawable
import androidx.fragment.app.viewModels
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_rent.databinding.FragmentRentPriceBinding
import com.drtaa.feature_rent.viewmodel.RentPriceViewModel
import com.drtaa.feature_rent.viewmodel.RentViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach

@AndroidEntryPoint
class RentPriceFragment : BaseFragment<FragmentRentPriceBinding>(R.layout.fragment_rent_price) {

    private val rentViewModel: RentViewModel by hiltNavGraphViewModels(R.id.nav_graph_rent)
    private val rentPriceViewModel: RentPriceViewModel by viewModels()

    private lateinit var selectedBackground: Drawable
    private lateinit var unSelectedBackground: Drawable

    override fun initView() {
        selectedBackground =
            getDrawable(requireActivity(), com.drtaa.core_ui.R.drawable.rect_blue_rad30)!!
        unSelectedBackground =
            getDrawable(requireActivity(), com.drtaa.core_ui.R.drawable.rect_white_rad30)!!

        initEvent()
        initObserve()
    }

    private fun initObserve() {
        rentPriceViewModel.rentIsHour.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isHour ->
                if (isHour == null) {
                    binding.btnRentPriceNext.isEnabled = false
                    return@onEach
                }

                binding.btnRentPriceNext.isEnabled = true
                if (isHour) {
                    binding.clRentPriceHour.background = selectedBackground
                    binding.clRentPriceDay.background = unSelectedBackground
                } else {
                    binding.clRentPriceHour.background = unSelectedBackground
                    binding.clRentPriceDay.background = selectedBackground
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initEvent() {
        binding.clRentPriceHour.setOnClickListener {
            rentPriceViewModel.setRentIsHour(true)
        }

        binding.clRentPriceDay.setOnClickListener {
            rentPriceViewModel.setRentIsHour(false)
        }

        binding.btnRentPriceNext.setOnClickListener {
            rentViewModel.setRentIsHour(rentPriceViewModel.rentIsHour.value!!)
            navigateDestination(R.id.action_rentPriceFragment_to_rentSummaryFragment)
        }
    }
}
