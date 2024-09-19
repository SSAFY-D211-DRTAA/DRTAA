package com.drtaa.feature_payment.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.PaymentRepository
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_model.data.PaymentCompletionInfo
import com.drtaa.core_model.data.SocialUser
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.firstOrNull
import kotlinx.coroutines.launch
import org.json.JSONObject
import timber.log.Timber
import java.util.Date
import javax.inject.Inject

@HiltViewModel
class PaymentViewModel @Inject constructor(
    private val paymentRepository: PaymentRepository,
    private val signRepository: SignRepository
) : ViewModel() {

    private val _paymentStatus = MutableSharedFlow<PaymentStatus>()
    val paymentStatus: SharedFlow<PaymentStatus> = _paymentStatus

    private val _currentUser = MutableStateFlow<SocialUser?>(null)
    val currentUser: StateFlow<SocialUser?> = _currentUser

    init {
        currentUser()
    }

    private fun currentUser() {
        viewModelScope.launch {
            signRepository.getUserData().collect { result ->
                result.onSuccess { currentUser ->
                    _currentUser.emit(currentUser)
                }.onFailure { error ->
                    Timber.e("사용자 정보를 가져오는데 실패했습니다...")
                    _paymentStatus.emit(PaymentStatus.Error("사용자 정보를 가져오는데 실패했습니다: ${error.message}"))
                }
            }
        }
    }

        fun processBootpayPayment(data: String) {
            viewModelScope.launch {
                try {
                    val currentUser = _currentUser.value
                    if (currentUser == null) {
                        _paymentStatus.emit(PaymentStatus.Error("사용자 정보를 찾을 수 없습니다. 다시 로그인해 주세요."))
                        return@launch
                    }

                    val paymentInfo = parseBootpayData(data, currentUser)
                    val paymentRequest = paymentInfo.toPaymentRequest()
                    paymentRepository.savePaymentInfo(paymentRequest).collect { result ->
                        result.onSuccess {
                            _paymentStatus.emit(PaymentStatus.Success("결제가 성공적으로 처리되었습니다."))
                        }.onFailure { error ->
                            _paymentStatus.emit(PaymentStatus.Error("결제 저장 중 오류가 발생했습니다: ${error.message}"))
                        }
                    }
                } catch (e: Exception) {
                    _paymentStatus.emit(PaymentStatus.Error("결제 데이터 파싱 중 오류가 발생했습니다: ${e.message}"))
                }
            }
        }

    private fun parseBootpayData(data: String,currentUser: SocialUser): PaymentCompletionInfo {
        val jsonObject = JSONObject(data)
        val dataObject = jsonObject.getJSONObject("data")

        val price = dataObject.getInt("price")
        val count = price / 50
        Timber.d("결제 금액: $price, 수량: $count")

        return PaymentCompletionInfo(
            receiptId = dataObject.getString("receipt_id"),
            orderId = dataObject.getString("order_id"),
            price = dataObject.getInt("price"),
            paymentMethod = dataObject.getString("method_origin"),
            purchasedAt = Date(),
            userId = currentUser.id,
            carId = 1,
            headCount = count, // 수량을 headCount로 사용
            rentStartTime = Date(),
            rentEndTime = Date(System.currentTimeMillis() + 3600000)
        )
    }

    fun getUserPayments() {
        viewModelScope.launch {
            paymentRepository.getUserPayments().collect { result ->
                result.onSuccess { payments ->
                    _paymentStatus.emit(PaymentStatus.PaymentInfoRetrieved(payments.firstOrNull() ?: return@collect))
                }.onFailure { error ->
                    _paymentStatus.emit(PaymentStatus.Error("결제 내역 조회 중 오류가 발생했습니다: ${error.message}"))
                }
            }
        }
    }

    sealed class PaymentStatus {
        data class Success(val message: String) : PaymentStatus()
        data class Error(val message: String) : PaymentStatus()
        data class PaymentInfoRetrieved(val paymentInfo: PaymentCompletionInfo) : PaymentStatus()
    }

}
