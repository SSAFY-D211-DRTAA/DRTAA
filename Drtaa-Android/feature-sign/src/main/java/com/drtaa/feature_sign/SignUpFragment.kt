package com.drtaa.feature_sign

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Color
import android.net.Uri
import android.os.Environment
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.content.ContextCompat
import androidx.core.widget.addTextChangedListener
import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.core_ui.checkValidPassword
import com.drtaa.core_ui.showSnackBar
import com.drtaa.feature_sign.databinding.FragmentSignUpBinding
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import java.io.File
import java.io.FileOutputStream
import java.security.SecureRandom

@AndroidEntryPoint
class SignUpFragment : BaseFragment<FragmentSignUpBinding>(R.layout.fragment_sign_up) {

    private val signUpFragmentViewModel: SignUpFragmentViewModel by viewModels()

    private val getImageLauncher =
        registerForActivityResult(ActivityResultContracts.GetContent()) { uri: Uri? ->
            uri?.let { handleImage(it) }
        }

    override fun initView() {
        initButtonEvent()
        initEditTextEvent()
        initObserver()
    }

    private fun initButtonEvent() {
        binding.btnSignUpIdChkDuplicated.setOnClickListener {
            signUpFragmentViewModel.checkDuplicatedId(binding.etSignUpId.text.toString())
        }

        binding.btnSignUp.setOnClickListener {
            signUpFragmentViewModel.signUp(
                id = binding.etSignUpId.text.toString(),
                pw = binding.etSignUpPw.text.toString(),
                nickname = binding.etSignUpNickname.text.toString()
            )
        }

        binding.cvSignUpProfile.setOnClickListener {
            openImagePicker()
        }
    }

    private fun initEditTextEvent() {
        binding.etSignUpId.addTextChangedListener { _ ->
            signUpFragmentViewModel.setIsDuplicatedId(null)
        }

        binding.etSignUpPw.addTextChangedListener { text ->
            val inputText = text.toString()

            signUpFragmentViewModel.setIsValidPw(
                if (inputText.isEmpty()) null else checkValidPassword(inputText)
            )

            signUpFragmentViewModel.setIsEqualPw(
                if (inputText.isEmpty() && binding.etSignUpChkPw.text.toString().isEmpty()) {
                    null
                } else {
                    inputText == binding.etSignUpChkPw.text.toString()
                }
            )
        }

        binding.etSignUpChkPw.addTextChangedListener { text ->
            val inputText = text.toString()
            signUpFragmentViewModel.setIsEqualPw(
                if (inputText.isEmpty() && binding.etSignUpPw.text.toString().isEmpty()) {
                    null
                } else {
                    inputText == binding.etSignUpPw.text.toString()
                }
            )
        }

        binding.etSignUpNickname.addTextChangedListener { text ->
            val inputText = text.toString()
            signUpFragmentViewModel.setIsEmptyNickname(inputText.isEmpty())
        }
    }

    private fun openImagePicker() {
        getImageLauncher.launch("image/*")
    }

    private fun handleImage(imageUri: Uri) {
        val imageFile = uriToFile(requireActivity(), imageUri)

        signUpFragmentViewModel.setProfileImage(imageUri, imageFile)
    }

    private fun uriToFile(context: Context, uri: Uri): File {
        val contentResolver = context.contentResolver
        val file =
            File(
                context.getExternalFilesDir(Environment.DIRECTORY_PICTURES),
                "${SecureRandom.getInstanceStrong().nextDouble()}.jpg"
            )

        contentResolver.openInputStream(uri)?.use { inputStream ->
            FileOutputStream(file).use { outputStream ->
                val buffer = ByteArray(IMAGE_SIZE)
                var length: Int
                while (inputStream.read(buffer).also { length = it } > 0) {
                    outputStream.write(buffer, 0, length)
                }
            }
        }
        return file
    }

    private fun initObserver() {
        observeSignUpSuccess()
        observeDuplicatedId()
        observeProfileImageUri()
        observePasswordValidation()
        observePasswordConfirmation()
        observeNicknameValidation()
        observeSignUpPossibility()
    }

    private fun observeSignUpSuccess() {
        signUpFragmentViewModel.isSignUpSuccess.observeWithLifecycle { isSignUpSuccess ->
            if (isSignUpSuccess) navigatePopBackStack() else showSnackBar("회원가입 실패")
        }
    }

    private fun observeDuplicatedId() {
        signUpFragmentViewModel.isDuplicatedId.observeWithLifecycle { isDuplicatedId ->
            updateIdHelpText(isDuplicatedId)
            signUpFragmentViewModel.setIsPossibleSignUp()
        }
    }

    private fun observeProfileImageUri() {
        signUpFragmentViewModel.profileImageUri.observeWithLifecycle { imageUri ->
            binding.ivSignUpProfile.setImageURI(imageUri)
        }
    }

    private fun observePasswordValidation() {
        signUpFragmentViewModel.isValidPw.observeWithLifecycle { isValidPw ->
            binding.tvSignUpPwHelp.text = if (isValidPw == false) "비밀번호 형식을 확인해주세요." else ""
            signUpFragmentViewModel.setIsPossibleSignUp()
        }
    }

    private fun observePasswordConfirmation() {
        signUpFragmentViewModel.isEqualPw.observeWithLifecycle { isEqualPw ->
            binding.tvSignUpChkPwHelp.text = if (isEqualPw == false) "비밀번호가 일치하지 않습니다." else ""
            signUpFragmentViewModel.setIsPossibleSignUp()
        }
    }

    private fun observeNicknameValidation() {
        signUpFragmentViewModel.isEmptyNickname.observeWithLifecycle { isEmptyNickname ->
            binding.tvSignUpNicknameHelp.text = if (isEmptyNickname) "닉네임을 설정해주세요." else ""
        }
    }

    private fun observeSignUpPossibility() {
        signUpFragmentViewModel.isPossibleSignUp.observeWithLifecycle { isPossibleSignUp ->
            binding.btnSignUp.isEnabled = isPossibleSignUp
        }
    }

    @SuppressLint("ResourceType")
    private fun updateIdHelpText(isDuplicatedId: Boolean?) {
        binding.tvSignUpIdHelp.apply {
            when (isDuplicatedId) {
                null -> {
                    text = ""
                    setTextColor(ContextCompat.getColor(context, Color.BLACK))
                }
                true -> {
                    text = "이미 사용중인 아이디입니다."
                    setTextColor(ContextCompat.getColor(context, Color.RED))
                }
                false -> {
                    text = "사용 가능한 아이디입니다."
                    setTextColor(ContextCompat.getColor(context, Color.BLUE))
                }
            }
        }
    }

    private fun <T> Flow<T>.observeWithLifecycle(action: (T) -> Unit) {
        flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach(action)
            .launchIn(viewLifecycleOwner.lifecycleScope)
    }

    companion object {
        const val IMAGE_SIZE = 1024
    }
}