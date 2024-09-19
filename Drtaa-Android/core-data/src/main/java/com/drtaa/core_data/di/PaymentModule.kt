package com.drtaa.core_data.di

import com.drtaa.core_data.datasource.PaymentDataSource
import com.drtaa.core_data.datasourceimpl.PaymentDataSourceImpl
import com.drtaa.core_data.repository.PaymentRepository
import com.drtaa.core_data.repositoryimpl.PaymentRepositoryImpl
import dagger.Binds
import dagger.Module
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent

@Module
@InstallIn(SingletonComponent::class)
abstract class PaymentModule {
    @Binds
    abstract fun bindPaymentRepository(
        paymentRepositoryImpl: PaymentRepositoryImpl
    ): PaymentRepository

    @Binds
    abstract fun bindPaymentDataSource(
        paymentDataSourceImpl: PaymentDataSourceImpl
    ): PaymentDataSource

}