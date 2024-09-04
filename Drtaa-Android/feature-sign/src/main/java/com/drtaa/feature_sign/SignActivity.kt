package com.drtaa.feature_sign

import androidx.activity.viewModels
import com.drtaa.core_ui.base.BaseActivity
import com.drtaa.feature_sign.databinding.ActivitySignBinding
import com.navercorp.nid.NaverIdLoginSDK
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class SignActivity : BaseActivity<ActivitySignBinding>(R.layout.activity_sign) {

    private val signViewModel: SignViewModel by viewModels()

    override fun init() {

    }

}