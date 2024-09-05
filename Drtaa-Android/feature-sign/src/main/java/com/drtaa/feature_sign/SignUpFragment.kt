package com.drtaa.feature_sign

import androidx.fragment.app.activityViewModels
import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_sign.databinding.FragmentSignUpBinding
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber

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
            signUpFragmentViewModel.checkDuplicatedId(binding.signUpIdEt.text.toString())
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

            }.launchIn(viewLifecycleOwner.lifecycleScope)

        signUpFragmentViewModel.isDuplicatedId.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isDuplicatedId ->
                if (isDuplicatedId) {
                    Timber.d("옵저버")
                    binding.signUpIdHelpTv.text = "이미 사용중인 아이디입니다."
                } else {
                    binding.signUpIdHelpTv.text = "사용 가능한 아이디입니다."
                }

            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

}