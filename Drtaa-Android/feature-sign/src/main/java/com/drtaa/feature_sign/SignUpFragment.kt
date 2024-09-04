package com.drtaa.feature_sign

import androidx.fragment.app.activityViewModels
import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_sign.databinding.FragmentSignUpBinding
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.onEach

@AndroidEntryPoint
class SignUpFragment : BaseFragment<FragmentSignUpBinding>(R.layout.fragment_sign_up) {

    private val signViewModel: SignViewModel by activityViewModels()
    private val signUpFragmentViewModel: SignUpFragmentViewModel by viewModels()

    override fun initView() {
        initEvent()
        initObserver()
    }

    private fun initEvent() {
        binding.signUpIdChkBtn.setOnClickListener {

        }

        binding.signUpNicknameChkBtn.setOnClickListener {

        }

        binding.signUpBtn.setOnClickListener {
            signUpFragmentViewModel.signUp(
                id = binding.signUpIdEt.text.toString(),
                pw = binding.signUpPwEt.text.toString(),
                nickname = binding.signUpNicknameEt.text.toString()
            )
        }
    }

    private fun initObserver() {
        signUpFragmentViewModel.isSignUpSuccess.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isSignUpSuccess ->
                if (isSignUpSuccess) {
                    navigatePopBackStack()
                } else {

                }

            }
    }

}