package com.drtaa.core_data

import com.drtaa.core_data.datasource.NaverDataSource
import com.drtaa.core_data.repositoryimpl.NaverRepositoryImpl
import com.drtaa.core_model.network.ResponseSearch
import io.mockk.coEvery
import io.mockk.mockk
import junit.framework.TestCase.assertEquals
import junit.framework.TestCase.assertTrue
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.test.runTest
import org.junit.Before
import org.junit.Test

@ExperimentalCoroutinesApi
class NaverRepositoryImplTest {

    private lateinit var repository: NaverRepositoryImpl
    private lateinit var dataSource: NaverDataSource

    @Before
    fun setup() {
        dataSource = mockk()
        repository = NaverRepositoryImpl(dataSource)
    }

    @Test
    fun `getSearchList success`() = runTest {
        val result = repository.getSearchList("구미").first()
        print(result)
    }

    @Test
    fun `getSearchList generic error`() = runTest {
        val keyword = "test"
        coEvery { dataSource.getSearchList(keyword) } throws Exception("Generic error")

        // When
        val result = repository.getSearchList(keyword).first()
        // Then
        print(result)
        assertTrue(result.isFailure)
        assertEquals("Generic error", result.exceptionOrNull()?.message)
    }
}