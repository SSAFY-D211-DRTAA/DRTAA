package com.drtaa.core_data

import com.drtaa.core_data.datasource.SignDataSource
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_model.data.SocialUser
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
import javax.inject.Inject


@HiltAndroidTest
@Config(application = HiltTestApplication::class, manifest = Config.NONE)
@RunWith(RobolectricTestRunner::class)
class SignRepositoryTest {
    @get:Rule
    var hiltRule = HiltAndroidRule(this)

    @Inject
    lateinit var signRepository: SignRepository

    @Inject
    lateinit var signDataSource: SignDataSource

    @Before
    fun setup() {
        hiltRule.inject()
        clearAllMocks()
    }

    @Test
    fun `setDataStore success`() = runBlocking {
        // Given
        val mockResponse = SocialUser(
            userLogin = "mingyu",
            id = "mingyu",
            name = "mingyu",
            nickname = "mingyu",
            profileImageUrl = null,
            accessToken = null,
            refreshToken = null
        )
        coEvery {
            signDataSource.setUserData(
                SocialUser(
                    userLogin = "mingyu",
                    id = "mingyu",
                    name = "mingyu",
                    nickname = "mingyu",
                    profileImageUrl = null,
                    accessToken = null,
                    refreshToken = null
                )
            )
            signDataSource.getUserData()
        } returns mockResponse

        // When
        val result = signRepository.getUserData().toList()

        // Then
        assertTrue(result.size == 1)
        assertTrue(result[0].isSuccess)
        val user = result.first().getOrNull()
        assertEquals("mingyu", user?.name)
        assertEquals("mingyu", user?.id)
        assertEquals("mingyu", user?.userLogin)
    }
}