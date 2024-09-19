package com.drtaa.core_data

import com.drtaa.core_data.datasource.NaverDataSource
import com.drtaa.core_data.repository.NaverRepository
import com.drtaa.core_model.network.ResponseSearch
import com.drtaa.core_model.network.SearchItem
import dagger.hilt.android.testing.HiltAndroidRule
import dagger.hilt.android.testing.HiltAndroidTest
import dagger.hilt.android.testing.HiltTestApplication
import io.mockk.clearAllMocks
import io.mockk.coEvery
import junit.framework.TestCase.assertEquals
import junit.framework.TestCase.assertTrue
import kotlinx.coroutines.flow.toList
import kotlinx.coroutines.runBlocking
import org.junit.Before
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config
import java.time.LocalDate
import javax.inject.Inject

@HiltAndroidTest
@Config(application = HiltTestApplication::class, manifest = Config.NONE)
@RunWith(RobolectricTestRunner::class)
class NaverRepositoryTest {

    @get:Rule
    var hiltRule = HiltAndroidRule(this)

    @Inject
    lateinit var naverRepository: NaverRepository

    @Inject
    lateinit var naverDataSource: NaverDataSource

    @Before
    fun setup() {
        hiltRule.inject()
        clearAllMocks()
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