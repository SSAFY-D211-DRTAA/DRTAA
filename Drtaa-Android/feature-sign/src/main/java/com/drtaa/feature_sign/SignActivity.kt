package com.drtaa.feature_sign

import android.content.Intent
import android.os.Bundle
import androidx.activity.viewModels
import androidx.core.splashscreen.SplashScreen.Companion.installSplashScreen
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.navigation.fragment.NavHostFragment
import com.drtaa.core_ui.base.BaseActivity
import com.drtaa.feature_main.MainActivity
import com.drtaa.feature_sign.databinding.ActivitySignBinding
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber

@AndroidEntryPoint
class SignActivity : BaseActivity<ActivitySignBinding>(R.layout.activity_sign) {

    private val signViewModel: SignViewModel by viewModels()

    override fun onCreate(savedInstanceState: Bundle?) {
        installSplashScreen()
        super.onCreate(savedInstanceState)
    }

    override fun init() {
        signViewModel
    }
}