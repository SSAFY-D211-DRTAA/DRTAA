package com.drtaa.feature_rent

import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.core_ui.component.OneButtonMessageDialog
import com.drtaa.core_ui.showSnackBar
import com.drtaa.feature_rent.databinding.FragmentRentBinding
import com.drtaa.feature_rent.viewmodel.RentViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber

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

        rentViewModel.rentStartSchedule.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { rentStartSchedule ->
                if (rentStartSchedule != null) {
                    binding.tvRentStartSchedule.text = rentStartSchedule.toString()
                } else {
                    binding.tvRentStartSchedule.hint = "09.09 (월) 20:00"
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        rentViewModel.rentEndSchedule.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { rentEndSchedule ->
                if (rentEndSchedule != null) {
                    binding.tvRentEndSchedule.text = rentEndSchedule.toString()
                } else {
                    binding.tvRentEndSchedule.hint = "09.09 (월) 20:00"
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        rentViewModel.rentPeople.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { rentPeople ->
                binding.tvRentPeople.text = rentPeople.toString()
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        rentViewModel.isRentValid.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isRentValid ->
                binding.btnRentNext.isEnabled = isRentValid
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        rentViewModel.isDuplicatedSchedule.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isDuplicatedSchedule ->
                Timber.d("isDuplicatedSchedule: $isDuplicatedSchedule")
                if (isDuplicatedSchedule == null) {
                    OneButtonMessageDialog(
                        requireActivity(),
                        "오류가 발생했습니다.\n" + "다시 시도해주세요."
                    ).show()
                } else if (isDuplicatedSchedule) {
                    OneButtonMessageDialog(
                        requireActivity(), "해당 일정에 중복된 예약이 존재합니다.\n다른 일정으로 다시 시도해주세요."
                    ).show()
                } else {
                    navigateDestination(R.id.action_rentFragment_to_rentSummaryFragment)
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

        binding.ivRentIncreasePeople.setOnClickListener {
            val isValid = rentViewModel.increaseRentPeople()
            if (isValid.not()) {
                showSnackBar("최대 인원은 8명입니다.")
            }
        }

        binding.ivRentDecreasePeople.setOnClickListener {
            val isValid = rentViewModel.decreaseRentPeople()
            if (isValid.not()) {
                showSnackBar("최소 인원은 1명입니다.")
            }
        }

        binding.btnRentNext.setOnClickListener {
            rentViewModel.checkDuplicatedSchedule()
        }
    }
}
