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
import dagger.hilt.android.AndroidEntryPoint

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
    }

}