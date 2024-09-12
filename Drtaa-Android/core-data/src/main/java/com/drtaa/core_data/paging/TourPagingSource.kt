package com.drtaa.core_data.paging

import androidx.paging.PagingSource
import androidx.paging.PagingState
import com.drtaa.core_data.datasource.TourDataSource
import com.drtaa.core_data.util.ResultWrapper
import com.drtaa.core_data.util.safeApiCall
import com.drtaa.core_model.data.TourItem
import com.drtaa.core_model.network.ResponseTour
import com.drtaa.core_model.util.toEntity
import timber.log.Timber

class TourPagingSource(
    private val tourDataSource: TourDataSource,
    private val mapX: String,
    private val mapY: String,
    private val radius: String
) : PagingSource<Int, TourItem>() {

    override fun getRefreshKey(state: PagingState<Int, TourItem>): Int? {
        return state.anchorPosition?.let { anchorPosition ->
            state.closestPageToPosition(anchorPosition)?.prevKey?.plus(1)
                ?: state.closestPageToPosition(anchorPosition)?.nextKey?.minus(1)
        }
    }

    override suspend fun load(params: LoadParams<Int>): LoadResult<Int, TourItem> {
        val nextPage = params.key ?: 1
        val response = tourDataSource.getLocationBasedList(
            numOfRows = params.loadSize,
            pageNo = nextPage,
            mapX = mapX,
            mapY = mapY,
            radius = radius
        ).response.body

        return when (val result = safeApiCall { response }) {
            is ResultWrapper.Success -> {
                val entity = result.data.items.item.map { it.toEntity() }
                Timber.tag("tour_pager").d("${result.data}")
                LoadResult.Page(
                    data = entity,
                    prevKey = if (nextPage == 1) null else nextPage - 1,
                    nextKey = if (isNextExist(response, result, nextPage)) null else nextPage + 1
                )
            }

            is ResultWrapper.GenericError -> {
                Timber.tag("tour_pager").e(result.message, "다음 페이지에서 뭐 없음 $nextPage")
                LoadResult.Error(Exception(result.message))
            }

            ResultWrapper.NetworkError -> {
                Timber.tag("tour_pager").e("Network Error")
                LoadResult.Error(Exception("Network Error"))
            }
        }
    }

    private fun isNextExist(
        response: ResponseTour.Response.Body,
        result: ResultWrapper.Success<ResponseTour.Response.Body>,
        nextPage: Int
    ): Boolean {
        return response.items.item.isEmpty() || result.data.pageNo == nextPage - 1
    }
}