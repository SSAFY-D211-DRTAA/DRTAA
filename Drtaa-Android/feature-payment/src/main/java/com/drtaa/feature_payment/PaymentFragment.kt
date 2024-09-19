package com.drtaa.feature_payment

import android.util.Log
import androidx.fragment.app.viewModels
import kr.co.bootpay.android.*
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_payment.databinding.FragmentPaymentBinding
import com.drtaa.feature_payment.viewmodel.PaymentViewModel
import dagger.hilt.android.AndroidEntryPoint
import kr.co.bootpay.android.events.BootpayEventListener
import kr.co.bootpay.android.models.BootExtra
import kr.co.bootpay.android.models.BootItem
import kr.co.bootpay.android.models.BootUser
import kr.co.bootpay.android.models.Payload
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_data.datasourceimpl.SignDataSourceImpl
import kotlinx.coroutines.flow.firstOrNull
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@AndroidEntryPoint
class PaymentFragment : BaseFragment<FragmentPaymentBinding>(R.layout.fragment_payment) {

    @Inject
    lateinit var signDataSourceImpl: SignDataSourceImpl

    private val viewModel: PaymentViewModel by viewModels()

    private var count = 1
    private val pricePerItem = 50

    private var applicationId = "66e2dabea3175898bd6e4b23" // 이건 왜 대체 적용 안됨? ;;; 어2가 아리마셍요 ㅋㅋ

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

        lifecycleScope.launch {
           val currentUser = signDataSourceImpl.getUserData()
            Timber.d("최근 유저 확인: $currentUser")

        }

        observeViewModel()
    }

    private fun updateCountAndPrice() {
        binding.tvPaymentCount.text = count.toString()
        binding.tvPaymentTotalprice.text = "총 금액: ${count * pricePerItem}원"
    }

    private fun observeViewModel() {
        viewModel.paymentStatus
            .onEach { status ->
                when (status) {
                    is PaymentViewModel.PaymentStatus.Success -> {
                        binding.tvPaymentStatus.text = "성공: ${status.message}"
                    }

                    is PaymentViewModel.PaymentStatus.Error -> {
                        binding.tvPaymentStatus.text = "오류: ${status.message}"
                    }

                    is PaymentViewModel.PaymentStatus.PaymentInfoRetrieved -> {
                        binding.tvPaymentStatus.text = "ㅇㅇㅇ"
                    }
                }
            }
            .launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private suspend fun requestbootPayment() {
        val bootUser = getBootUser()

        val extra = BootExtra()
            .setCardQuota("0,2,3")

        val items = ArrayList<BootItem>().apply {
            add(BootItem().setName("아이템").setId("ITEM_CODE").setQty(count).setPrice(pricePerItem.toDouble()))
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

        val map: MutableMap<String, Any> = HashMap()
        map["1"] = "abcdef"
        map["2"] = "abcdef55"
        map["3"] = 1234
        payload.metadata = map

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

    private suspend fun getBootUser(): BootUser? {
        val currentUser = signDataSourceImpl.getUserData()
        return BootUser().apply {
            id = currentUser.id
            username = currentUser.nickname
            email = "abc@naver.com"
            phone = ""
            area = ""
            gender = 0
        }
    }
}