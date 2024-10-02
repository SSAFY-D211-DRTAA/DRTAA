package com.drtaa.core_data.di

import com.drtaa.core_data.datasource.NaverDataSource
import com.drtaa.core_data.datasource.PlanDataSource
import com.drtaa.core_data.datasource.RentCarDataSource
import com.drtaa.core_data.datasource.RentDataSource
import com.drtaa.core_data.datasource.SignDataSource
import com.drtaa.core_data.datasource.TaxiDataSource
import com.drtaa.core_data.datasource.TokenDataSource
import com.drtaa.core_data.datasource.TourDataSource
import com.drtaa.core_data.datasource.TravelDataSource
import com.drtaa.core_data.datasourceimpl.NaverDataSourceImpl
import com.drtaa.core_data.datasourceimpl.PlanDataSourceImpl
import com.drtaa.core_data.datasourceimpl.RentCarDataSourceImpl
import com.drtaa.core_data.datasourceimpl.RentDataSourceImpl
import com.drtaa.core_data.datasourceimpl.SignDataSourceImpl
import com.drtaa.core_data.datasourceimpl.TaxiDataSourceImpl
import com.drtaa.core_data.datasourceimpl.TokenDataSourceImpl
import com.drtaa.core_data.datasourceimpl.TourDataSourceImpl
import com.drtaa.core_data.datasourceimpl.TravelDataSourceImpl
import dagger.Binds
import dagger.Module
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
interface DataSourceModule {
    @Singleton
    @Binds
    fun bindTokenDataSource(tokenDataSourceImpl: TokenDataSourceImpl): TokenDataSource

    @Singleton
    @Binds
    fun bindSignDataSource(signDataSourceImpl: SignDataSourceImpl): SignDataSource

    @Singleton
    @Binds
    fun bindMapDataSource(mapDataSourceImpl: NaverDataSourceImpl): NaverDataSource

    @Singleton
    @Binds
    fun bindTourDataSource(tourDataSourceImpl: TourDataSourceImpl): TourDataSource

    @Singleton
    @Binds
    fun bindRentDataSource(rentDataSourceImpl: RentDataSourceImpl): RentDataSource

    @Singleton
    @Binds
    fun bindRentCarDataSource(rentCarDataSourceImpl: RentCarDataSourceImpl): RentCarDataSource

    @Singleton
    @Binds
    fun bindTaxiDataSource(taxiDataSourceImpl: TaxiDataSourceImpl): TaxiDataSource

    @Singleton
    @Binds
    fun bindPlanDataSource(planDataSourceImpl: PlanDataSourceImpl): PlanDataSource

    @Singleton
    @Binds
    fun bindTravelDataSource(travelDataSourceImpl: TravelDataSourceImpl): TravelDataSource
}