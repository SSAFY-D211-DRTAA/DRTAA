package com.drtaa.feature_mypage.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.PaymentRepository
import com.drtaa.core_model.pay.PaymentCompletionInfo
import com.drtaa.core_model.pay.ResponsePayment
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@HiltViewModel
class PaymentListViewModel @Inject constructor(
    private val paymentRepository: PaymentRepository
) : ViewModel() {

    private val _paymentList = MutableStateFlow<List<ResponsePayment>>(emptyList())
    val paymentList: StateFlow<List<ResponsePayment>> = _paymentList

    init {
        getPaymentList()
    }

    fun getPaymentList() {
        viewModelScope.launch {
            paymentRepository.getUserPayments().collect { result ->
                result.onSuccess { paymentList ->
                    Timber.d("받은 리스트는요: $paymentList")
                    _paymentList.value = paymentList
                    Timber.d("받은 밸류: ${_paymentList.value}")
                }.onFailure { error ->
                    Timber.e(error, "결제 내역 조회 오류")
                    _paymentList.value = emptyList()
                }
            }
        }
    }
}