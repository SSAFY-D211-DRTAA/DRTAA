package com.drtaa.feature_mypage.viewmodel

import android.net.Uri
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_data.repository.TokenRepository
import com.drtaa.core_model.sign.SocialUser
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.launch
import timber.log.Timber
import java.io.File
import javax.inject.Inject

@HiltViewModel
class MyPageViewModel @Inject constructor(
    private val signRepository: SignRepository,
    private val tokenRepository: TokenRepository,
) : ViewModel() {

    private val _currentUser = MutableStateFlow<SocialUser?>(null)
    val currentUser: StateFlow<SocialUser?> = _currentUser

    private val _updateResult = MutableSharedFlow<Boolean>()
    val updateResult: SharedFlow<Boolean> = _updateResult

    private val _profileImageUri = MutableStateFlow<Uri?>(null)
    val profileImageUri: StateFlow<Uri?> = _profileImageUri

    private val _profileImageFile = MutableStateFlow<File?>(null)
    private val profileImageFile: StateFlow<File?> = _profileImageFile

    private val _logoutState = MutableSharedFlow<Boolean>()
    val logoutState: SharedFlow<Boolean> = _logoutState

    init {
        getUserData()
    }

    private fun getUserData() {
        viewModelScope.launch {
            signRepository.getUserData().collect { result ->
                result.onSuccess { user ->
                    _currentUser.value = user
                }.onFailure {
                    Timber.e("유저 정보 조회 오류")
                }
            }
        }
    }

    fun setProfileImage(imageUri: Uri?, imageFile: File?) {
        viewModelScope.launch {
            _profileImageUri.value = imageUri
            _profileImageFile.value = imageFile

            updateProfileImage()
        }
    }

    private fun updateProfileImage() {
        viewModelScope.launch {
            profileImageFile.value.let { file ->
                signRepository.updateProfileImage(file).collect { result ->
                    result.onSuccess { updatedUser ->
                        _updateResult.emit(true)
                        _currentUser.value = updatedUser
                        _profileImageUri.value = Uri.parse(updatedUser.profileImageUrl)
                        val userData = signRepository.getUserData().first()
                        Timber.d("$userData 마이페이지 바뀌고 유저")
                    }.onFailure {
                        _updateResult.emit(false)
                        Timber.d("유저 이미지 업데이트 실패")
                    }
                }
            }
        }
    }

    fun logout() {
        viewModelScope.launch {
            signRepository.clearUserData()
            tokenRepository.clearToken().collect {
                it.onSuccess {
                    _logoutState.emit(true)
                }.onFailure {
                    _logoutState.emit(false)
                }
            }
        }
    }
}