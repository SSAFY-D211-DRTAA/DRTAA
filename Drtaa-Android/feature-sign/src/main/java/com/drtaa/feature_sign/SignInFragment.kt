package com.drtaa.feature_sign

import android.content.Intent
import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_main.MainActivity
import com.drtaa.feature_sign.databinding.FragmentSignInBinding
import com.navercorp.nid.NaverIdLoginSDK
import com.navercorp.nid.oauth.OAuthLoginCallback
import dagger.hilt.android.AndroidEntryPoint
import timber.log.Timber

@AndroidEntryPoint
class SignInFragment : BaseFragment<FragmentSignInBinding>(R.layout.fragment_sign_in) {

    override fun initView() {
        initEvent()
    }

    private fun initEvent(){
        binding.signinMoveToMainBtn.setOnClickListener {
            startActivity(Intent(requireContext(), MainActivity::class.java))
            requireActivity().finish()
        }

        binding.signinNaverBtn.setOnClickListener {
            NaverIdLoginSDK.authenticate(requireActivity(), oAuthLoginCallback)
        }
    }

    val oAuthLoginCallback = object : OAuthLoginCallback {
        override fun onError(errorCode: Int, message: String) {
            onFailure(errorCode, message)
        }

        override fun onFailure(httpStatus: Int, message: String) {
            val errorCode = NaverIdLoginSDK.getLastErrorCode().code
            val errorDescription = NaverIdLoginSDK.getLastErrorDescription()
            Timber.tag("fail naver login").d("error code $errorCode, error description $errorDescription")
        }

        override fun onSuccess() {
            Timber.tag("success naver login").d("AccessToken : ${NaverIdLoginSDK.getAccessToken()}")
            Timber.tag("success naver login").d("ReFreshToken : ${NaverIdLoginSDK.getRefreshToken()}")
            Timber.tag("success naver login").d("Expires  : ${NaverIdLoginSDK.getExpiresAt()}")
            Timber.tag("success naver login").d("TokenType   : ${NaverIdLoginSDK.getTokenType()}")
            Timber.tag("success naver login").d("State   : ${NaverIdLoginSDK.getState()}")

        }
    }

}