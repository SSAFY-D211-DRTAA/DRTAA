package com.drtaa.feature_mypage

import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_mypage.adaper.PaymentListAdapter
import com.drtaa.feature_mypage.databinding.FragmentPaymentListBinding
import com.drtaa.feature_mypage.viewmodel.PaymentListViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach

@AndroidEntryPoint
class PaymentListFragment : BaseFragment<FragmentPaymentListBinding>(R.layout.fragment_payment_list) {

    private val paymentListViewModel: PaymentListViewModel by viewModels()

    private val paymentListAdapter = PaymentListAdapter()

    override fun onResume() {
        super.onResume()
        paymentListViewModel.getPaymentList()
    }

    override fun initView() {
        initObserve()
        initRVAdapter()
    }

    private fun initRVAdapter() {
        binding.rvPaymentList.adapter = paymentListAdapter
    }

    private fun initObserve() {
        paymentListViewModel.paymentList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { paymentList ->
                    paymentListAdapter.submitList(paymentList)
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }
}