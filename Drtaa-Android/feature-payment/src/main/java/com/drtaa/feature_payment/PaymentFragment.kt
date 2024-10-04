package com.drtaa.feature_payment

import android.os.Bundle
import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.navigation.fragment.findNavController
import androidx.navigation.fragment.navArgs
import com.drtaa.core_model.rent.Payment
import com.drtaa.core_model.sign.SocialUser
import com.drtaa.core_model.util.Pay
import com.drtaa.core_ui.base.BaseDialogFragment
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
import timber.log.Timber

@AndroidEntryPoint
class PaymentFragment : BaseDialogFragment<FragmentPaymentBinding>(R.layout.fragment_payment) {

    private val viewModel: PaymentViewModel by viewModels()
    private val args: PaymentFragmentArgs by navArgs()
    private var isPaymentCompleted = false

    override fun initView(savedInstanceState: Bundle?) {
        observeViewModel()
    }

    private fun observeViewModel() {
        viewModel.currentUser.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach { user ->
            user?.let {
                requestPayment(user, args.payment)
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun requestPayment(user: SocialUser, payment: Payment) {
        val bootUser = getBootUser(user)
        val extra = BootExtra()
            .setCardQuota("0,2,3")
        val product = payment.items.first()
        val items = ArrayList<BootItem>().apply {
            add(
                BootItem().setName(product.name).setId(product.id).setQty(product.quantity)
//                    .setPrice(product.price.toDouble() - SALE)
                    .setPrice(Defalut_Price)
            )
        }

        val payload = Payload().apply {
            setApplicationId(BuildConfig.BOOTPAY_APP_ID)
            setOrderName(payment.orderName)
            setOrderId(payment.orderId)
            setPrice(Defalut_Price)
            setUser(bootUser)
            setExtra(extra)
            this.items = items
        }

        Bootpay.init(childFragmentManager, requireContext())
            .setPayload(payload)
            .setEventListener(object : BootpayEventListener {
                override fun onCancel(data: String) {
                    Timber.tag("bootpay").d("cancel: %s", data)
                    findNavController().previousBackStackEntry?.savedStateHandle?.set(
                        Pay.CANCELED.type,
                        true
                    )
                    dismiss()
                }

                override fun onError(data: String) {
                    Timber.tag("bootpay").d("error: %s", data)
                    findNavController().previousBackStackEntry?.savedStateHandle?.set(
                        Pay.CLOSED.type,
                        true
                    )
                    dismiss()
                }

                override fun onClose() {
                    Timber.tag("bootpay").d("onClose")
                    if (!isPaymentCompleted) {
                        findNavController().previousBackStackEntry?.savedStateHandle?.set(
                            Pay.CANCELED.type,
                            true
                        )
                        Bootpay.removePaymentWindow()
                        dismiss()
                    }
                }

                override fun onIssued(data: String) {
                    Timber.tag("bootpay").d("issued: %s", data)
                }

                override fun onConfirm(data: String): Boolean {
                    Timber.tag("bootpay").d("confirm: %s", data)
                    return true
                }

                override fun onDone(data: String) {
                    Timber.tag("bootpay").d("done: %s", data)
                    isPaymentCompleted = true
                    viewLifecycleOwner.lifecycleScope.launch {
                        Bootpay.removePaymentWindow()
                        findNavController().previousBackStackEntry?.savedStateHandle?.set(
                            Pay.SUCCESS.type,
                            Pair(true, data)
                        )
                        val action = PaymentFragmentDirections.actionPaymentFragmentToPaymentDoneFragment(args.payment)
                        findNavController().navigate(action)
                        Timber.d("나 실행되긴 하는겨?")
                    }
                }
            }).requestPayment()
    }

    private fun getBootUser(user: SocialUser): BootUser {
        return BootUser().apply {
            username = user.nickname // naver 로그인 시 nickname이 유효
        }
    }

    companion object {
        const val SALE = 239900.0
        const val Defalut_Price = 100.0
    }
}