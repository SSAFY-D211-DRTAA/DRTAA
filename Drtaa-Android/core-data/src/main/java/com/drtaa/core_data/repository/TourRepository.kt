package com.drtaa.core_data.repository

import androidx.paging.PagingData
import com.drtaa.core_model.tour.TourItem
import kotlinx.coroutines.flow.Flow

interface TourRepository {
    suspend fun getLocationBasedList(
        mapX: String,
        mapY: String,
        radius: String,
    ): Flow<Result<Flow<PagingData<TourItem>>>>
}