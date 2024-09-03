package com.drtaa.feature_sign

import android.content.Intent
import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_main.MainActivity
import com.drtaa.feature_sign.databinding.FragmentSignInBinding
import com.drtaa.feature_sign.util.NaverLoginManager
import com.navercorp.nid.NaverIdLoginSDK
import com.navercorp.nid.oauth.OAuthLoginCallback
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber

@AndroidEntryPoint
class SignInFragment : BaseFragment<FragmentSignInBinding>(R.layout.fragment_sign_in) {

    override fun initView() {
        initEvent()
        initObserver()
    }

    private fun initEvent() {
        binding.signinMoveToMainBtn.setOnClickListener {
            startActivity(Intent(requireContext(), MainActivity::class.java))
            requireActivity().finish()
        }

        binding.signinNaverBtn.setOnClickListener {
            NaverLoginManager.login(requireActivity())
        }
    }

    private fun initObserver() {
        NaverLoginManager.resultLogin.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { result ->
                result.onSuccess { data ->
                    Timber.tag("login success").d("$data")
                    startActivity(Intent(requireContext(), MainActivity::class.java))
                    requireActivity().finish()
                }.onFailure {
                    Timber.tag("login fail").d("$result")
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

}