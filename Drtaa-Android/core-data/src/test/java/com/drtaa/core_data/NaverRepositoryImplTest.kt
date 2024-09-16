package com.drtaa.core_data

import com.drtaa.core_data.datasource.NaverDataSource
import com.drtaa.core_data.repositoryimpl.NaverRepositoryImpl
import com.drtaa.core_model.network.ResponseSearch
import com.drtaa.core_model.network.SearchItem
import io.mockk.coEvery
import io.mockk.mockk
import junit.framework.TestCase.assertEquals
import junit.framework.TestCase.assertTrue
import kotlinx.coroutines.flow.toList
import kotlinx.coroutines.runBlocking
import org.junit.Before
import org.junit.Test
import java.time.LocalDate

class NaverRepositoryImplTest {

    private lateinit var naverDataSource: NaverDataSource
    private lateinit var naverRepository: NaverRepositoryImpl

    @Before
    fun setup() {
        naverDataSource = mockk()
        naverRepository = NaverRepositoryImpl(naverDataSource)
    }

    @Test
    fun `getSearchList success html tag remove`() = runBlocking {
        // Given
        val keyword = "test"
        val mockResponse = ResponseSearch(
            display = 0,
            lastBuildDate = LocalDate.now().toString(),
            start = 0,
            total = 10,
            items = listOf(
                SearchItem(
                    title = "<b>제목이요</b>",
                    link = "https://test.com",
                    category = "Test Category",
                    description = "Test Description",
                    telephone = "123-456-7890",
                    address = "Test Address",
                    roadAddress = "Test Road Address",
                    mapx = "123",
                    mapy = "456"
                )
            )
        )
        coEvery { naverDataSource.getSearchList(keyword) } returns mockResponse

        // When
        val result = naverRepository.getSearchList(keyword).toList()

        // Then
        assertTrue(result.size == 1)
        assertTrue(result[0].isSuccess)
        val searchList = result.first().getOrNull()
        assertEquals(1, searchList?.size)
        assertEquals("제목이요", searchList?.first()?.title)
    }

    @Test
    fun `getSearchList network error`() = runBlocking {
        // Given
        val keyword = "test"
        coEvery { naverDataSource.getSearchList(keyword) } throws Exception("네트워크 에러")

        // When
        val result = naverRepository.getSearchList(keyword).toList()

        // Then
        assertTrue(result.size == 1)
        assertTrue(result[0].isFailure)
        assertEquals("네트워크 에러", result[0].exceptionOrNull()?.message)
    }

    @Test
    fun `getSearchList generic error`() = runBlocking {
        // Given
        val keyword = "test"
        val errorMessage = "Generic error occurred"
        coEvery { naverDataSource.getSearchList(keyword) } throws Exception(errorMessage)

        // When
        val result = naverRepository.getSearchList(keyword).toList()

        // Then
        assertTrue(result.size == 1)
        assertTrue(result[0].isFailure)
        assertEquals(errorMessage, result[0].exceptionOrNull()?.message)
    }
}