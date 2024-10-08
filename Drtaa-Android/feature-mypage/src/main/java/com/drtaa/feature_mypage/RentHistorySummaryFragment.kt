package com.drtaa.feature_mypage

import android.view.View
import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.navigation.fragment.navArgs
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_mypage.databinding.FragmentRentHistorySummaryBinding
import com.drtaa.feature_mypage.viewmodel.RentHistorySummaryViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach

@AndroidEntryPoint
class RentHistorySummaryFragment :
    BaseFragment<FragmentRentHistorySummaryBinding>(R.layout.fragment_rent_history_summary) {

    private val rentSummaryViewModel: RentHistorySummaryViewModel by viewModels()
    private val args by navArgs<RentHistorySummaryFragmentArgs>()

    override fun initView() {
        showLoading()
        initObserve()
        initData()
    }

    private fun initData() {
        rentSummaryViewModel.setRentId(args.rentId)
    }

    private fun initObserve() {
        rentSummaryViewModel.rentSummary.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { rentSummary ->
                if (rentSummary == null) {
                    binding.svSummary.visibility = View.GONE
                    binding.tvSummaryError.visibility = View.VISIBLE
                } else {
                    dismissLoading()
                    binding.rentDetail = rentSummary
                    binding.originalPrice = (rentSummary.rentTime * PRICE_PER_HOUR).toInt()
                    binding.disCountPrice =
                        -1 * ((rentSummary.rentTime.toInt() / ONE_DAY) * DISCOUNT_PER_DAY)

                    binding.svSummary.visibility = View.VISIBLE
                    binding.tvSummaryError.visibility = View.GONE
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    companion object {
        private const val PRICE_PER_HOUR = 20000
        private const val DISCOUNT_PER_DAY = 240000
        private const val ONE_DAY = 24
    }
}
