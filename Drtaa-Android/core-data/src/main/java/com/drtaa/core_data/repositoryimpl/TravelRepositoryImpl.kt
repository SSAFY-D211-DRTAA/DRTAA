package com.drtaa.core_data.repositoryimpl

import com.drtaa.core_data.datasource.TravelDataSource
import com.drtaa.core_data.repository.TravelRepository
import com.drtaa.core_data.util.ResultWrapper
import com.drtaa.core_data.util.safeApiCall
import com.drtaa.core_model.travel.NaverImage
import com.drtaa.core_model.travel.NaverPost
import com.drtaa.core_model.travel.Weather
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import timber.log.Timber
import javax.inject.Inject

class TravelRepositoryImpl @Inject constructor(
    private val travelDataSource: TravelDataSource
) : TravelRepository {
    override suspend fun getBlogPostList(keyword: String): Flow<Result<List<NaverPost>>> = flow {
        when (
            val response = safeApiCall { travelDataSource.getBlogPostList(keyword) }
        ) {
            is ResultWrapper.Success -> {
                emit(Result.success(response.data))
                Timber.d("블로그 포스트 가져오기 성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.e("블로그 포스트 가져오기 실패")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 오류")))
                Timber.e("네트워크 오류")
            }
        }
    }

    override suspend fun getWeatherList(lat: Double, lon: Double): Flow<Result<List<Weather>>> =
        flow {
            when (
                val response = safeApiCall { travelDataSource.getWeatherList(lat, lon) }
            ) {
                is ResultWrapper.Success -> {
                    emit(Result.success(response.data))
                    Timber.d("날씨 가져오기 성공")
                }

                is ResultWrapper.GenericError -> {
                    emit(Result.failure(Exception(response.message)))
                    Timber.e("날씨 가져오기 실패")
                }

                is ResultWrapper.NetworkError -> {
                    emit(Result.failure(Exception("네트워크 오류")))
                    Timber.e("네트워크 오류")
                }
            }
        }

    override suspend fun getImage(keyword: String): Flow<Result<NaverImage>> = flow {
        when (
            val response = safeApiCall { travelDataSource.getImageList(keyword) }
        ) {
            is ResultWrapper.Success -> {
                val imageList =
                    response.data.items.sortedBy { it.sizeheight.toDouble() / it.sizewidth.toDouble() }

                Timber.d("$imageList")

                emit(Result.success(imageList.first()))
                Timber.d("이미지 가져오기 성공")
            }

            is ResultWrapper.GenericError -> {
                emit(Result.failure(Exception(response.message)))
                Timber.e("이미지 가져오기 실패")
            }

            is ResultWrapper.NetworkError -> {
                emit(Result.failure(Exception("네트워크 오류")))
                Timber.e("네트워크 오류")
            }
        }
    }
}
