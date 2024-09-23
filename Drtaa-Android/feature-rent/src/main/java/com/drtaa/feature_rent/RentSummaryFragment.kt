package com.drtaa.feature_rent

import android.util.Log
import android.view.View
import android.widget.Toast
import androidx.fragment.app.viewModels
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.sign.SocialUser
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_rent.databinding.FragmentRentSummaryBinding
import com.drtaa.feature_rent.viewmodel.RentSummaryViewModel
import com.drtaa.feature_rent.viewmodel.RentViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kr.co.bootpay.android.Bootpay
import kr.co.bootpay.android.events.BootpayEventListener
import kr.co.bootpay.android.models.BootExtra
import kr.co.bootpay.android.models.BootItem
import kr.co.bootpay.android.models.BootUser
import kr.co.bootpay.android.models.Payload

@AndroidEntryPoint
class RentSummaryFragment :
    BaseFragment<FragmentRentSummaryBinding>(R.layout.fragment_rent_summary) {

    private val rentViewModel: RentViewModel by hiltNavGraphViewModels(R.id.nav_graph_rent)
    private val rentSummaryViewModel: RentSummaryViewModel by viewModels()

    override fun initView() {
        showLoading()

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
                        Toast.makeText(requireContext(), status.message, Toast.LENGTH_SHORT).show()
                    }
                    is RentSummaryViewModel.PaymentStatus.Error -> {
                        Toast.makeText(requireContext(), status.message, Toast.LENGTH_SHORT).show()
                    }
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun requestBootpayPayment() {
        val rentInfo = rentViewModel.rentInfo.value ?: return
        val currentUser = rentSummaryViewModel.currentUser.value ?: return

        val user = getBootUser(currentUser)

        val extra = BootExtra().setCardQuota("0,2,3")

        val items = ArrayList<BootItem>().apply {
            add(
                BootItem().setName("DRTAA 렌트")
                .setId("RENT_CODE")
                .setQty(1)
                .setPrice(rentInfo.finalPrice.toDouble() - sale)
            )
        }

        val payload = Payload().apply {
            setApplicationId("66e2dabea3175898bd6e4b23")
            setOrderName("DRTAA 렌트 이용")
            setOrderId(System.currentTimeMillis().toString())
            setPrice(rentInfo.finalPrice.toDouble() - sale)
            setUser(user)
            setExtra(extra)
            this.items = items
        }

        Bootpay.init(childFragmentManager, requireContext())
            .setPayload(payload)
            .setEventListener(object : BootpayEventListener {
                override fun onCancel(data: String) {
                    Log.d("bootpay", "cancel: $data")
                }

                override fun onError(data: String) {
                    Log.d("bootpay", "error: $data")
                }

                override fun onClose() {
                    Bootpay.removePaymentWindow()
                }

                override fun onIssued(data: String) {
                    Log.d("bootpay", "issued: $data")
                }

                override fun onConfirm(data: String): Boolean {
                    Log.d("bootpay", "confirm: $data")
                    return true
                }

                override fun onDone(data: String) {
                    Log.d("bootpay", "done: $data")
                    rentSummaryViewModel.processBootpayPayment(data, rentInfo)
                }
            }).requestPayment()
    }

    private fun getBootUser(currentUser: SocialUser): BootUser {
        return BootUser().apply {
            id = currentUser.id
            username = currentUser.name
            email = "user@example.com"
            phone = "010-1234-5678"
        }
    }

    private fun initEvent() {
        binding.btnSummaryPay.setOnClickListener {
            requestBootpayPayment()
        }
    }

    companion object {
        const val sale = 239900.0
    }
}
