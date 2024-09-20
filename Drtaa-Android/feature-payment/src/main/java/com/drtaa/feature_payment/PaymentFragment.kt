package com.drtaa.feature_payment

import android.util.Log
import androidx.fragment.app.viewModels
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_payment.databinding.FragmentPaymentBinding
import com.drtaa.feature_payment.viewmodel.PaymentViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.launch
import kr.co.bootpay.android.Bootpay
import kr.co.bootpay.android.events.BootpayEventListener
import kr.co.bootpay.android.models.BootExtra
import kr.co.bootpay.android.models.BootItem
import kr.co.bootpay.android.models.BootUser
import kr.co.bootpay.android.models.Payload

@AndroidEntryPoint
class PaymentFragment : BaseFragment<FragmentPaymentBinding>(R.layout.fragment_payment) {

    private val viewModel: PaymentViewModel by viewModels()

    private var count = 1
    private val pricePerItem = 50

    override fun initView() {
        binding.btnPaymentBootpay.setOnClickListener {
            lifecycleScope.launch {
                requestbootPayment()
            }
        }

        binding.btnPaymentCntDec.setOnClickListener {
            if (count > 1) {
                count--
                updateCountAndPrice()
            }
        }
        binding.btnPaymentCntInc.setOnClickListener {
            count++
            updateCountAndPrice()
        }

        updateCountAndPrice()
        observeViewModel()
    }

    private fun updateCountAndPrice() {
        binding.tvPaymentCount.text = count.toString()
        binding.tvPaymentTotalprice.text = "총 금액: ${count * pricePerItem}원"
    }

    private fun observeViewModel() {
        viewModel.paymentStatus
            .onEach { status ->
                binding.tvPaymentStatus.text = when (status) {
                    is PaymentViewModel.PaymentStatus.Success -> {
                        "성공: ${status.message}"
                    }

                    is PaymentViewModel.PaymentStatus.Error -> {
                        "오류: ${status.message}"
                    }

                    is PaymentViewModel.PaymentStatus.PaymentInfoRetrieved -> {
                        "ㅇㅇㅇ"
                    }
                }
            }
            .launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun requestbootPayment() {
        val bootUser = getBootUser()

        val extra = BootExtra()
            .setCardQuota("0,2,3")

        val items = ArrayList<BootItem>().apply {
            add(
                BootItem().setName("아이템").setId("ITEM_CODE").setQty(count)
                    .setPrice(pricePerItem.toDouble())
            )
        }

        val payload = Payload().apply {
            setApplicationId("66e2dabea3175898bd6e4b23")
            setOrderName("부트페이 결제테스트")
            setOrderId("1234")
            setPrice((count * pricePerItem).toDouble())
            setUser(bootUser)
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
                    viewModel.processBootpayPayment(data)
                }
            }).requestPayment()
    }

    private fun getBootUser(): BootUser? {
        return viewModel.currentUser.value?.let { currentUser ->
            BootUser().apply {
                id = currentUser.id
                username = currentUser.nickname
                email = "abc@naver.com"
                phone = ""
                area = ""
                gender = 0
            }
        }
    }
}