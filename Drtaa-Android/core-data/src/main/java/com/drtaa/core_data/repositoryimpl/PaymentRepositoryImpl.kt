package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.datasource.PaymentDataSource
import com.drtaa.core_data.repository.PaymentRepository
import com.drtaa.core_data.util.ResultWrapper
import com.drtaa.core_data.util.safeApiCall
import com.drtaa.core_model.pay.PaymentCompletionInfo
import com.drtaa.core_model.network.RequestPayment
import com.drtaa.core_model.pay.ResponsePayment
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import timber.log.Timber
import javax.inject.Inject

class PaymentRepositoryImpl @Inject constructor(
    private val paymentDataSource: PaymentDataSource
) : PaymentRepository {
    override suspend fun savePaymentInfo(requestPayment: RequestPayment): Flow<Result<Unit>> =
        flow {
            when (val response = safeApiCall { paymentDataSource.savePaymentInfo(requestPayment) }) {
                is ResultWrapper.Success -> {
                    emit(Result.success(Unit))
                    Timber.d("결제 정보 저장 성공!")
                }

                is ResultWrapper.GenericError -> {
                    emit(Result.failure(Exception(response.message)))
                    Timber.d("결제 정보 저장 실패!")
                }

                is ResultWrapper.NetworkError -> {
                    emit(Result.failure(Exception("네트워크 에러")))
                    Timber.d("네트워크 에러")
                }
            }
        }

    override suspend fun getPaymentInfo(receiptId: String): Flow<Result<PaymentCompletionInfo>> =
        flow {
            when (val response = safeApiCall { paymentDataSource.getPaymentInfo(receiptId) }) {
                is ResultWrapper.Success -> {
                    emit(Result.success(response.data))
                    Timber.d("결제 정보 조회 성공!")
                }

                is ResultWrapper.GenericError -> {
                    emit(Result.failure(Exception(response.message)))
                    Timber.d("결제 정보 조회 실패!")
                }

                is ResultWrapper.NetworkError -> {
                    emit(Result.failure(Exception("네트워크 에러")))
                    Timber.d("네트워크 에러")
                }
            }
        }

    override suspend fun getUserPayments(): Flow<Result<List<ResponsePayment>>> =
        flow {
            when (val response = safeApiCall { paymentDataSource.getUserPayments() }) {
                is ResultWrapper.Success -> {
                    emit(Result.success(response.data))
                    Timber.d("사용자 결제 내역 조회 성공")
                }

                is ResultWrapper.GenericError -> {
                    emit(Result.failure(Exception(response.message)))
                    Timber.d("사용자 결제 내역 조회 실패")
                }

                is ResultWrapper.NetworkError -> {
                    emit(Result.failure(Exception("네트워크 에러")))
                    Timber.d("네트워크 에러")
                }
            }
        }
}
