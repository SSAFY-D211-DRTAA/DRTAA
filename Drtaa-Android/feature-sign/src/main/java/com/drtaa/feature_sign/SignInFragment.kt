package com.drtaa.feature_sign

import android.content.Intent
import android.provider.Settings
import androidx.credentials.CredentialManager
import androidx.credentials.CustomCredential
import androidx.credentials.GetCredentialRequest
import androidx.credentials.GetCredentialResponse
import androidx.credentials.exceptions.GetCredentialException
import androidx.fragment.app.activityViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_main.MainActivity
import com.drtaa.feature_sign.databinding.FragmentSignInBinding
import com.drtaa.feature_sign.util.NaverLoginManager
import com.google.android.gms.common.api.ApiException
import com.google.android.libraries.identity.googleid.GetGoogleIdOption
import com.google.android.libraries.identity.googleid.GoogleIdTokenCredential
import com.google.android.libraries.identity.googleid.GoogleIdTokenParsingException
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@AndroidEntryPoint
class SignInFragment : BaseFragment<FragmentSignInBinding>(R.layout.fragment_sign_in) {

    private val signViewModel: SignViewModel by activityViewModels()

    @Inject
    lateinit var googleIdOption: GetGoogleIdOption

    override fun initView() {
        initEvent()
        initObserver()
    }

    private fun initEvent() {
        binding.btnSignInMoveToMain.setOnClickListener {
            startActivity(Intent(requireContext(), MainActivity::class.java))
            requireActivity().finish()
        }

        binding.btnSignInNaver.setOnClickListener {
            NaverLoginManager.login(requireActivity())
        }

        binding.btnSignIn.setOnClickListener {
            signViewModel.formLogin(
                id = binding.etSignInId.text.toString(),
                pw = binding.etSignInPw.text.toString()
            )
        }

        binding.btnSignInSignUp.setOnClickListener {
            navigateDestination(R.id.action_signInFragment_to_signUpFragment)
        }

        binding.btnSignInGoogle.setOnClickListener {
            CoroutineScope(Dispatchers.IO).launch {
                googleLogin()
            }
        }
    }

    private fun initObserver() {
        NaverLoginManager.resultLogin.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { result ->
                result.onSuccess { data ->
                    Timber.tag("login success").d("$data")
                    signViewModel.getTokens(data)
                }.onFailure {
                    Timber.tag("login fail").d("$result")
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        signViewModel.tokens.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { result ->
                result.onSuccess {
                    startActivity(Intent(requireContext(), MainActivity::class.java))
                    requireActivity().finish()
                }.onFailure {}
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private suspend fun googleLogin() {
        val credentialManager = CredentialManager.create(requireActivity())

        val request: GetCredentialRequest = GetCredentialRequest.Builder()
            .addCredentialOption(googleIdOption)
            .build()

        coroutineScope {
            try {
                val result = credentialManager.getCredential(
                    request = request,
                    context = requireActivity()
                )
                handleSignIn(result)
            } catch (e: GetCredentialException) {
                if(e.type == android.credentials.GetCredentialException.TYPE_NO_CREDENTIAL){
                    startActivity(Intent(Settings.ACTION_ADD_ACCOUNT))
                }
            }
        }
    }

    private fun handleSignIn(result: GetCredentialResponse) {
        when (val credential = result.credential) {
            // 구글 아이디 토큰
            is CustomCredential -> {
                if (credential.type == GoogleIdTokenCredential.TYPE_GOOGLE_ID_TOKEN_CREDENTIAL) {
                    try {
                        val googleIdTokenCredential = GoogleIdTokenCredential
                            .createFrom(credential.data)
                        Timber.d("id : ${googleIdTokenCredential.id}")
                        Timber.d("idToken : ${googleIdTokenCredential.idToken}")
                    } catch (e: GoogleIdTokenParsingException) {
                        Timber.d("Received an invalid google id token response", e)
                    }
                } else {
                    Timber.d("Unexpected type of credential")
                }
            }
        }
    }
}