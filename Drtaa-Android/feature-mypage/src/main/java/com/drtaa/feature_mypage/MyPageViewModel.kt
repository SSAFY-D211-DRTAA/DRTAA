package com.drtaa.feature_mypage

import androidx.lifecycle.ViewModel
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class MyPageViewModel @Inject constructor(

) : ViewModel() {
    val someText = "testtest"
}