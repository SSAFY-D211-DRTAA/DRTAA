package com.drtaa.feature_taxi

import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.navigation.fragment.findNavController
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.core_ui.component.OneButtonMessageDialog
import com.drtaa.core_ui.showSnackBar
import com.drtaa.feature_taxi.databinding.FragmentTaxiBinding
import com.drtaa.feature_taxi.viewmodel.TaxiViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber

@AndroidEntryPoint
class TaxiFragment : BaseFragment<FragmentTaxiBinding>(R.layout.fragment_taxi) {

    private val taxiViewModel: TaxiViewModel by hiltNavGraphViewModels(R.id.nav_graph_taxi)

    override fun initView() {
        initEvent()
        initObserve()
    }

    private fun initObserve() {
        taxiViewModel.taxiStartLocation.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { taxiStartLocation ->
                if (taxiStartLocation != null) {
                    binding.tvTaxiStartLocation.text = taxiStartLocation.title
                } else {
                    binding.tvTaxiStartLocation.hint = "상암 MBC"
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        taxiViewModel.taxiEndLocation.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { taxiEndLocation ->
                if (taxiEndLocation != null) {
                    binding.tvTaxiEndLocation.text = taxiEndLocation.title
                } else {
                    binding.tvTaxiEndLocation.hint = "상암 월드컵 경기장"
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        taxiViewModel.isDuplicatedSchedule.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isDuplicatedSchedule ->
                Timber.d("중복 확인: $isDuplicatedSchedule")
                if (isDuplicatedSchedule == null) {
                    OneButtonMessageDialog(
                        requireActivity(),
                        "오류가 발생했습니다.\n" + "다시 시도해주세요."
                    ).show()
                } else if (isDuplicatedSchedule) {
                    OneButtonMessageDialog(
                        requireActivity(), "현재 배정 가능한 차가 없습니다"
                    ).show()
                } else {
                    navigateDestination(R.id.action_taxiFragment_to_taxiSummaryFragment)
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initEvent() {
        binding.clTaxiStartLocation.setOnClickListener {
            navigationToSearch(isStartLocation = true)
        }

        binding.clTaxiEndLocation.setOnClickListener {
            navigationToSearch(isStartLocation = false)
        }

        binding.btnTaxiNext.setOnClickListener {
            if(taxiViewModel.taxiStartLocation.value == null || taxiViewModel.taxiEndLocation.value==null) {
                showSnackBar("장소를 선택해주세요!")
            } else {
                taxiViewModel.checkDuplicatedSchedule()
            }

        }
    }

    private fun navigationToSearch(isStartLocation: Boolean) {
        val action = TaxiFragmentDirections.actionTaxiFragmentToTaxiSearchFragment(isStartLocation)
        findNavController().navigate(action)
    }
}