package com.drtaa.feature_mypage

import androidx.lifecycle.ViewModel
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class MyPageViewModel @Inject constructor(
    // 뭐 들어갈지 생각하기
) : ViewModel() {
    val someText = "testtest"
}