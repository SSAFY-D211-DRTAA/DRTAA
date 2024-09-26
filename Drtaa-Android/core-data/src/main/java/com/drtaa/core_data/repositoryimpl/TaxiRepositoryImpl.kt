package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.datasource.TaxiDataSource
import com.drtaa.core_data.repository.TaxiRepository
import javax.inject.Inject

class TaxiRepositoryImpl @Inject constructor(
    private val taxiDataSource: TaxiDataSource,
) : TaxiRepository{
}