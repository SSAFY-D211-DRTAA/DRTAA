package com.drtaa.feature_sign

import com.drtaa.core_ui.base.BaseActivity
import com.drtaa.feature_sign.databinding.ActivitySignBinding
import com.navercorp.nid.NaverIdLoginSDK
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class SignActivity : BaseActivity<ActivitySignBinding>(R.layout.activity_sign) {
    override fun init() {
        initNaverSign()
    }

    private fun initNaverSign(){
        NaverIdLoginSDK.initialize(
            this,
            "",
            "",
            "사용자",
        )
    }
}