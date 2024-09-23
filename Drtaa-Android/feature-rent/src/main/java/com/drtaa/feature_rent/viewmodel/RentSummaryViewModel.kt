package com.drtaa.feature_rent.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.PaymentRepository
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_model.data.PaymentCompletionInfo
import com.drtaa.core_model.map.Search
import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.rent.RentCar
import com.drtaa.core_model.rent.RentInfo
import com.drtaa.core_model.sign.SocialUser
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.launch
import org.json.JSONObject
import timber.log.Timber
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import javax.inject.Inject

@HiltViewModel
class RentSummaryViewModel @Inject constructor(
    private val rentRepository: RentRepository,
    private val paymentRepository: PaymentRepository,
    private val signRepository: SignRepository,
) : ViewModel() {

    private val _assignedCar = MutableSharedFlow<RentCar?>()
    val assignedCar: SharedFlow<RentCar?> = _assignedCar

    private val _paymentStatus = MutableSharedFlow<PaymentStatus>()
    val paymentStatus: SharedFlow<PaymentStatus> = _paymentStatus

    private val _currentUser = MutableStateFlow<SocialUser?>(null)
    val currentUser: StateFlow<SocialUser?> = _currentUser

    private val _rentStartLocation = MutableStateFlow<Search?>(null)
    val rentStartLocation: StateFlow<Search?> = _rentStartLocation

    init {
        getCurrentUser()
    }

    fun setRentStartLocation(search: Search) {
        _rentStartLocation.value = search
    }

    private fun getCurrentUser() {
        viewModelScope.launch {
            signRepository.getUserData().collect { result ->
                result.onSuccess { user ->
                    _currentUser.value = user
                }.onFailure { error ->
                    Timber.e("유저 정보 조회 오류")
                }
            }
        }
    }

    fun getUnAssignedCar(rentSchedule: RequestUnassignedCar) {
        viewModelScope.launch {
            rentRepository.getUnassignedCar(rentSchedule).collect { result ->
                result.onSuccess { data ->
                    Timber.d("성공")
                    _assignedCar.emit(data)
                }.onFailure {
                    Timber.d("실패")
                    _assignedCar.emit(null)
                }
            }
        }
    }

    fun processBootpayPayment(paymentData: String, rentInfo: RentInfo) {
        viewModelScope.launch {
            runCatching {
                val currentUser = signRepository.getUserData().first().getOrThrow()
                val paymentInfo = parseBootpayData(paymentData, currentUser, rentInfo)
                val paymentRequest = paymentInfo.toPaymentRequest()
                paymentRepository.savePaymentInfo(paymentRequest).collect { result ->
                    result.onSuccess {
                        _paymentStatus.emit(PaymentStatus.Success("결제가 성공적으로 처리 되었습니다."))
                        callRent(rentInfo)
                    }.onFailure { error ->
                        _paymentStatus.emit(PaymentStatus.Error("결제 저장 중 오류가 발생했습니다: ${error.message}"))
                    }
                }
            }.onFailure { e ->
                _paymentStatus.emit(PaymentStatus.Error("결제 데이터 처리 중 오류가 발생했습니다: ${e.message}"))
            }
        }
    }

    private fun parseBootpayData(
        data: String,
        currentUser: SocialUser,
        rentInfo: RentInfo
    ): PaymentCompletionInfo {
        val jsonObject = JSONObject(data)
        val dataObject = jsonObject.getJSONObject("data")
        val dateFormat = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss", Locale.getDefault())

        return PaymentCompletionInfo(
            receiptId = dataObject.getString("receipt_id"),
            orderId = dataObject.getString("order_id"),
            price = dataObject.getInt("price"),
            paymentMethod = dataObject.getString("method_origin"),
            purchasedAt = dateFormat.parse(dataObject.getString("purchased_at")) ?: Date(),
            userId = currentUser.id,
            carId = rentInfo.carInfo?.rentCarId ?: 0,
            headCount = rentInfo.people,
            rentStartTime = rentInfo.startSchedule.toDate(),
            rentEndTime = rentInfo.endSchedule.toDate()
        )
    }

    private fun callRent(rentInfo: RentInfo) {
        viewModelScope.launch {
            val rentStartLocation = _rentStartLocation.value

            val dateFormatter = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss", Locale.getDefault())

            val requestCallRent = RequestCallRent(
                rentHeadCount = rentInfo.people,
                rentTime = rentInfo.hours.toInt(),
                rentPrice = rentInfo.finalPrice.toLong(),
                rentStartTime = dateFormatter.format(rentInfo.startSchedule.toDate()),
                rentEndTime = dateFormatter.format(rentInfo.endSchedule.toDate()),
                rentDptLat = rentStartLocation!!.lat,
                rentDptLon = rentStartLocation.lng
            )

            rentRepository.callRent(requestCallRent).collect { result ->
                result.onSuccess { response ->
                    Timber.d("호출 성공")
                }.onFailure { error ->
                    Timber.d("호출 실패")
                }
            }
        }
    }

    sealed class PaymentStatus {
        data class Success(val message: String) : PaymentStatus()
        data class Error(val message: String) : PaymentStatus()
    }
}