package com.drtaa.core_data.repositoryimpl

import androidx.paging.Pager
import androidx.paging.PagingConfig
import androidx.paging.PagingData
import com.drtaa.core_data.datasource.TourDataSource
import com.drtaa.core_data.paging.TourPagingSource
import com.drtaa.core_data.repository.TourRepository
import com.drtaa.core_model.tour.TourItem
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flowOf
import timber.log.Timber
import javax.inject.Inject

class TourRepositoryImpl @Inject constructor(
    private val tourDataSource: TourDataSource
) : TourRepository {
    override suspend fun getLocationBasedList(
        mapX: String,
        mapY: String,
        radius: String,
    ): Flow<Result<Flow<PagingData<TourItem>>>> {
        Timber.tag("tour").d("TourRepository call")
        try {
            return flowOf(
                Result.success(
                    Pager(
                        config = PagingConfig(
                            pageSize = PAGE_SIZE,
                            enablePlaceholders = false,
                            initialLoadSize = PAGE_SIZE
                        ),
                        pagingSourceFactory = {
                            TourPagingSource(
                                tourDataSource,
                                mapX,
                                mapY,
                                radius
                            )
                        }
                    ).flow
                )
            )
        } catch (e: Exception) {
            Timber.tag("tour").e(e, "Error fetching tour items")
            return flowOf(Result.failure(e))
        }
    }

    companion object {
        private const val PAGE_SIZE = 10
    }
}