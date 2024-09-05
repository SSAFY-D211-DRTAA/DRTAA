package com.drtaa.feature_sign

import android.net.Uri
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.drtaa.core_data.repository.SignRepository
import com.drtaa.core_model.network.RequestSignUp
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import timber.log.Timber
import java.io.File
import javax.inject.Inject

@HiltViewModel
class SignUpFragmentViewModel @Inject constructor(
    private val signRepository: SignRepository
) : ViewModel() {

    private val _isSignUpSuccess = MutableStateFlow(false)
    val isSignUpSuccess: StateFlow<Boolean> = _isSignUpSuccess

    private val _isDuplicatedId = MutableSharedFlow<Boolean?>()
    val isDuplicatedId: SharedFlow<Boolean?> = _isDuplicatedId

    private val _profileImageUri = MutableStateFlow<Uri?>(null)
    val profileImageUri: StateFlow<Uri?> = _profileImageUri

    private val _profileImageFile = MutableStateFlow<File?>(null)
    private val profileImageFile: StateFlow<File?> = _profileImageFile

    fun signUp(id: String, pw: String, nickname: String) {
        val requestSignUp = RequestSignUp(
            userProviderId = id,
            userPassword = pw,
            userNickname = nickname,
            userIsAdmin = false
        )

        viewModelScope.launch {
            signRepository.signUp(requestSignUp, profileImageFile.value).collect { result ->
                result.onSuccess {
                    _isSignUpSuccess.emit(true)
                }.onFailure {
                    _isSignUpSuccess.emit(false)
                }

            }
        }
    }

    fun checkDuplicatedId(id: String) {
        viewModelScope.launch {
            signRepository.checkDuplicatedId(id).collect { result ->
                result.onSuccess { data ->
                    Timber.d("중복 체크 $data")
                    _isDuplicatedId.emit(data)
                }.onFailure {
                    _isDuplicatedId.emit(true)
                }
            }
        }
    }

    fun setProfileImage(imageUri: Uri?, imageFile: File?) {
        viewModelScope.launch {
            _profileImageUri.emit(imageUri)
            _profileImageFile.emit(imageFile)
        }
    }

    fun setIsDuplicatedId(isDuplicatedId: Boolean?) {
        viewModelScope.launch {
            _isDuplicatedId.emit(isDuplicatedId)
        }

    }
}