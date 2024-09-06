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
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.launch
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

    private val _isValidPw = MutableStateFlow<Boolean?>(null)
    val isValidPw: StateFlow<Boolean?> = _isValidPw

    private val _isEqualPw = MutableStateFlow<Boolean?>(null)
    val isEqualPw: StateFlow<Boolean?> = _isEqualPw

    private val _isPossibleSignUp = MutableStateFlow(false)
    val isPossibleSignUp: StateFlow<Boolean> = _isPossibleSignUp

    private val _isEmptyNickname = MutableStateFlow(true)
    val isEmptyNickname: StateFlow<Boolean> = _isEmptyNickname

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
                    _isDuplicatedId.emit(data)
                }.onFailure {
                    _isDuplicatedId.emit(true)
                }
            }
        }
    }

    fun setProfileImage(imageUri: Uri?, imageFile: File?) {
        viewModelScope.launch {
            _profileImageUri.value = imageUri
            _profileImageFile.value = imageFile
        }
    }

    fun setIsDuplicatedId(isDuplicatedId: Boolean?) {
        viewModelScope.launch {
            _isDuplicatedId.emit(isDuplicatedId)
        }
    }

    fun setIsValidPw(isValidPw: Boolean?) {
        viewModelScope.launch {
            _isValidPw.value = isValidPw
        }
    }

    fun setIsEqualPw(isEqual: Boolean?) {
        viewModelScope.launch {
            _isEqualPw.value = isEqual
        }
    }

    fun setIsEmptyNickname(isEmptyNickname: Boolean) {
        viewModelScope.launch {
            _isEmptyNickname.value = isEmptyNickname
        }
    }

    fun setIsPossibleSignUp() {
        viewModelScope.launch {
            combine(
                isValidPw,
                isEqualPw,
                isDuplicatedId,
                isEmptyNickname
            ) { isValidPw, isEqualPw, isDuplicatedId, isEmptyNickname ->
                isValidPw == true && isEqualPw == true && isDuplicatedId == false && !isEmptyNickname
            }.collect { isPossibleSignUp ->
                _isPossibleSignUp.value = isPossibleSignUp
            }
        }
    }

}