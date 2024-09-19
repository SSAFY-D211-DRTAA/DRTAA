package com.drtaa.feature_rent

import android.view.View
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_rent.databinding.FragmentRentSummaryBinding
import com.drtaa.feature_rent.viewmodel.RentViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber

@AndroidEntryPoint
class RentSummaryFragment :
    BaseFragment<FragmentRentSummaryBinding>(R.layout.fragment_rent_summary) {

    private val rentViewModel: RentViewModel by hiltNavGraphViewModels(R.id.nav_graph_rent)

    override fun initView() {
        showLoading()

        initEvent()
        initObserve()

        rentViewModel.getRentInfo()
    }

    private fun initObserve() {
        rentViewModel.rentInfo.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { result ->
                if (result == null) {
                    return@onEach
                }

                result.onSuccess { data ->
                    dismissLoading()
                    Timber.tag("rent info success").d("$data")
                    if (data.isAvailable) {
                        binding.rentInfo = data
                        binding.svSummary.visibility = View.VISIBLE
                    } else {
                        binding.tvSummaryNoCar.visibility = View.VISIBLE
                    }
                }.onFailure {
                    dismissLoading()
                    Timber.tag("rent info fail").d("$result")
                    binding.tvSummaryError.visibility = View.VISIBLE
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initEvent() {
        binding.btnSummaryPay.setOnClickListener {
//
        }
    }
}
