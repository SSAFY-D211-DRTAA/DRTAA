package com.drtaa.feature_payment

import android.net.Uri
import androidx.navigation.NavDeepLinkRequest
import androidx.navigation.NavOptions
import androidx.navigation.fragment.findNavController
import androidx.navigation.fragment.navArgs
import com.drtaa.core_ui.DeepLinkConstants
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_payment.databinding.FragmentPaymentDoneBinding

class PaymentDoneFragment : BaseFragment<FragmentPaymentDoneBinding>(R.layout.fragment_payment_done) {

    private val args: PaymentDoneFragmentArgs by navArgs()

    private fun initEvent() {
        binding.btnPaymentDone.setOnClickListener {
            when (args.payment.orderId) {
                "1" -> navigateToPlanHistory()
                "2" -> navigateToHome()
            }
        }
    }

    private fun navigateToPlanHistory() {
        val request = NavDeepLinkRequest.Builder
            .fromUri(Uri.parse(DeepLinkConstants.PLAN_HISTORY))
            .build()
        findNavController().navigate(
            request,
            NavOptions.Builder()
                .setPopUpTo(findNavController().graph.startDestinationId, false)
                .build()
        )
    }

    private fun navigateToHome() {
        val request = NavDeepLinkRequest.Builder
            .fromUri(Uri.parse(DeepLinkConstants.HOME))
            .build()
        findNavController().navigate(
            request,
            NavOptions.Builder()
                .setPopUpTo(findNavController().graph.startDestinationId, true)
                .build()
        )
    }

    override fun initView() {
        initEvent()
    }
}