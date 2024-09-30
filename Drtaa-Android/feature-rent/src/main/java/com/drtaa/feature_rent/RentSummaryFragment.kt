package com.drtaa.feature_rent

import android.view.View
import androidx.fragment.app.viewModels
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.navigation.fragment.findNavController
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.rent.Payment
import com.drtaa.core_model.util.Pay
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.core_ui.showSnackBar
import com.drtaa.feature_rent.databinding.FragmentRentSummaryBinding
import com.drtaa.feature_rent.viewmodel.RentSummaryViewModel
import com.drtaa.feature_rent.viewmodel.RentViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.launch
import timber.log.Timber

@AndroidEntryPoint
class RentSummaryFragment :
    BaseFragment<FragmentRentSummaryBinding>(R.layout.fragment_rent_summary) {

    private val rentViewModel: RentViewModel by hiltNavGraphViewModels(R.id.nav_graph_rent)
    private val rentSummaryViewModel: RentSummaryViewModel by viewModels()

    override fun initView() {
        showLoading()
        observeDialog()
        initEvent()
        initObserve()
        initData()
    }

    private fun initData() {
        val rentStartLocation = rentViewModel.rentStartLocation.value
        if (rentStartLocation != null) {
            rentSummaryViewModel.setRentStartLocation(rentStartLocation)
        }

        val rentSchedule = RequestUnassignedCar(
            rentCarScheduleStartDate = rentViewModel.rentStartSchedule.value!!.toRequestUnassignedCar(),
            rentCarScheduleEndDate = rentViewModel.rentEndSchedule.value!!.toRequestUnassignedCar(),
        )
        rentSummaryViewModel.getUnAssignedCar(rentSchedule)
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
                    binding.rentCar = result
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

        rentSummaryViewModel.paymentStatus.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { status ->
                when (status) {
                    is RentSummaryViewModel.PaymentStatus.Success -> {
                        showSnackBar(status.message)
                    }

                    is RentSummaryViewModel.PaymentStatus.Error -> {
                        showSnackBar(status.message)
                    }
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initEvent() {
        binding.btnSummaryPay.setOnClickListener {
            val action = RentSummaryFragmentDirections.actionFragmentRentToNavGraphPayment(
                Payment(
                    "DRTAA 렌트 이용",
                    "1",
                    listOf(
                        Payment.Product(
                            "DRTAA 렌트",
                            "RENT_CODE",
                            rentViewModel.rentInfo.value!!.finalPrice,
                            1
                        )
                    )
                )
            )
            clearBackStackEntryState()
            navigateDestination(action)
        }
    }

    /**
     * 매번 savedStateHandle 초기화해주기
     */
    private fun clearBackStackEntryState() {
        val savedStateHandle = findNavController().currentBackStackEntry?.savedStateHandle ?: return

        val states = mapOf(
            Pay.SUCCESS.type to Pair(false, ""),
            Pay.CLOSED.type to false,
            Pay.CANCELED.type to false
        )

        states.forEach { (key, value) ->
            savedStateHandle[key] = value
        }
    }

    private fun observeDialog() {
        viewLifecycleOwner.lifecycleScope.launch {
            launch {
                findNavController()
                    .currentBackStackEntry
                    ?.savedStateHandle
                    ?.getStateFlow<Pair<Boolean, String>>(Pay.SUCCESS.type, Pair(false, ""))
                    ?.collectLatest { (success, paymentData) ->
                        if (success) {
                            showSnackBar("결제에 성공했습니다")
                            Timber.tag("bootpay").d("rent에용 ${Pay.SUCCESS.type}: $paymentData")
                            rentSummaryViewModel.processBootpayPayment(
                                paymentData,
                                rentViewModel.rentInfo.value!!
                            )
                        }
                    }
            }
            launch {
                findNavController()
                    .currentBackStackEntry
                    ?.savedStateHandle
                    ?.getStateFlow<Boolean>(Pay.CLOSED.type, false)
                    ?.collectLatest { closed ->
                        if (closed) {
                            showSnackBar("결제가 취소되었습니다")
                            Timber.tag("bootpay").d(Pay.CLOSED.type)
                        }
                    }
            }
            launch {
                findNavController()
                    .currentBackStackEntry
                    ?.savedStateHandle
                    ?.getStateFlow<Boolean>(Pay.CANCELED.type, false)
                    ?.collectLatest { canceled ->
                        if (canceled) {
                            showSnackBar("결제에 실패하였습니다")
                            Timber.tag("bootpay").d(Pay.CANCELED.type)
                        }
                    }
            }
        }
    }
}
