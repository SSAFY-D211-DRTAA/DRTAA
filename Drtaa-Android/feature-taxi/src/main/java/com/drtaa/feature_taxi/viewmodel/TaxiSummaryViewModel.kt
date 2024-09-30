package com.drtaa.feature_taxi.viewmodel

import android.provider.ContactsContract.Data
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.PaymentRepository
import com.drtaa.core_data.repository.RentCarRepository
import com.drtaa.core_data.repository.RentRepository
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_data.repository.TaxiRepository
import com.drtaa.core_model.data.PaymentCompletionInfo
import com.drtaa.core_model.map.Search
import com.drtaa.core_model.network.RequestCallRent
import com.drtaa.core_model.network.RequestDuplicatedSchedule
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.rent.RentCar
import com.drtaa.core_model.rent.RentInfo
import com.drtaa.core_model.rent.RentSchedule
import com.drtaa.core_model.route.RouteInfo
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
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
import java.util.Date
import java.util.Locale
import javax.inject.Inject

@HiltViewModel
class TaxiSummaryViewModel @Inject constructor(
    private val taxiRepository: TaxiRepository,
    private val rentCarRepository: RentCarRepository,
    private val signRepository: SignRepository,
    private val paymentRepository: PaymentRepository,
    private val rentRepository: RentRepository
) : ViewModel() {

    private val _paymentStatus = MutableSharedFlow<PaymentStatus>()
    val paymentStatus: SharedFlow<PaymentStatus> = _paymentStatus

    private val _assignedCar = MutableSharedFlow<RentCar?>()
    val assignedCar: SharedFlow<RentCar?> = _assignedCar

    private val _taxiStartLocation = MutableStateFlow<Search?>(null)
    val taxiStartLocation: StateFlow<Search?> = _taxiStartLocation

    fun setTaxiStartLocation(search: Search) {
        _taxiStartLocation.value = search
    }


    fun getUnAssignedCar(rentSchedule: RequestUnassignedCar) {
        viewModelScope.launch {
            rentCarRepository.getUnassignedCar(rentSchedule).collect { result ->
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

    fun processBootpayPayment(paymentData: String, taxiInfo: RentInfo) {
        viewModelScope.launch {
            runCatching {
                val currentUser = signRepository.getUserData().first().getOrThrow()
                val paymentInfo = parseBootpayData(paymentData,currentUser,taxiInfo)
                val paymentRequest = paymentInfo.toPaymentRequest()
                paymentRepository.savePaymentInfo(paymentRequest).collect{ result ->
                    result.onSuccess {
                        _paymentStatus.emit(PaymentStatus.Success("결제가 완료되었습니다."))
                        callRent(taxiInfo)
                    }.onFailure {
                        _paymentStatus.emit(PaymentStatus.Error("결제에 실패했습니다."))
                    }
                }
            }
        }
    }

    private fun parseBootpayData(
        data: String,
        currentUser: SocialUser,
        taxiInfo: RentInfo,
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
            carId = taxiInfo.carInfo?.rentCarId ?: 0,
            headCount = taxiInfo.people,
            rentStartTime = taxiInfo.startSchedule.toDate(),
            rentEndTime = taxiInfo.endSchedule.toDate()
        )
    }

    private fun callRent(rentInfo: RentInfo) {
        viewModelScope.launch {
            val taxiStartLocation = _taxiStartLocation.value

            val dateFormatter = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss", Locale.getDefault())

            val requestCallRent = RequestCallRent(
                rentHeadCount = rentInfo.people,
                rentTime = rentInfo.hours.toInt(),
                rentPrice = rentInfo.finalPrice.toLong(),
                rentStartTime = dateFormatter.format(rentInfo.startSchedule.toDate()),
                rentEndTime = dateFormatter.format(rentInfo.endSchedule.toDate()),
                rentDptLat = taxiStartLocation!!.lat,
                rentDptLon = taxiStartLocation.lng,
                datePlacesName = taxiStartLocation.title,
                datePlacesCategory = taxiStartLocation.category,
                datePlacesAddress = taxiStartLocation.roadAddress
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
