package com.drtaa.feature_sign

import android.content.Context
import android.net.Uri
import android.os.Environment
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.widget.addTextChangedListener
import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.core_ui.checkValidPassword
import com.drtaa.core_ui.showSnackBar
import com.drtaa.feature_sign.databinding.FragmentSignUpBinding
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import java.io.File
import java.io.FileOutputStream

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

        binding.signUpProfileCv.setOnClickListener {
            openImagePicker()
        }
    }

    private fun initEditTextEvent() {
        binding.signUpIdEt.addTextChangedListener { _ ->
            signUpFragmentViewModel.setIsDuplicatedId(null)
        }

        binding.signUpPwEt.addTextChangedListener { text ->
            val inputText = text.toString()

            signUpFragmentViewModel.setIsValidPw(
                if (inputText.isEmpty()) null else checkValidPassword(inputText)
            )

            signUpFragmentViewModel.setIsEqualPw(
                if (inputText.isEmpty() && binding.signUpChkPwEt.text.toString().isEmpty()) {
                    null
                } else {
                    inputText == binding.signUpChkPwEt.text.toString()
                }
            )
        }

        binding.signUpChkPwEt.addTextChangedListener { text ->
            val inputText = text.toString()
            signUpFragmentViewModel.setIsEqualPw(
                if (inputText.isEmpty() && binding.signUpPwEt.text.toString().isEmpty()) {
                    null
                } else {
                    inputText == binding.signUpPwEt.text.toString()
                }
            )
        }

        binding.signUpNicknameEt.addTextChangedListener { text ->
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
            File(context.getExternalFilesDir(Environment.DIRECTORY_PICTURES), "temp_image.jpg")

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
        signUpFragmentViewModel.isSignUpSuccess.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isSignUpSuccess ->
                if (isSignUpSuccess) {
                    navigatePopBackStack()
                } else {
                    showSnackBar("회원가입 실패,")
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        signUpFragmentViewModel.isDuplicatedId.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isDuplicatedId ->
                binding.signUpIdHelpTv.text = when (isDuplicatedId) {
                    null -> ""
                    true -> "이미 사용중인 아이디입니다."
                    false -> "사용 가능한 아이디입니다."
                }
                signUpFragmentViewModel.setIsPossibleSignUp()
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        signUpFragmentViewModel.profileImageUri.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { imageUri ->
                binding.signUpProfileIv.setImageURI(imageUri)
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        signUpFragmentViewModel.isValidPw.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isValidPw ->
                binding.signUpPwHelpTv.text = when (isValidPw) {
                    null -> ""
                    true -> ""
                    false -> "비밀번호 형식을 확인해주세요."
                }
                signUpFragmentViewModel.setIsPossibleSignUp()
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        signUpFragmentViewModel.isEqualPw.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isEqualPw ->
                binding.signUpChkPwHelpTv.text = when (isEqualPw) {
                    null -> ""
                    true -> ""
                    false -> "비밀번호가 일치하지 않습니다."
                }
                signUpFragmentViewModel.setIsPossibleSignUp()
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        signUpFragmentViewModel.isEmptyNickname.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isEmptyNickname ->
                binding.signUpNicknameHelpTv.text = if (isEmptyNickname) "닉네임을 설정해주세요." else ""
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        signUpFragmentViewModel.isPossibleSignUp.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isPossibleSignUp ->
                binding.signUpBtn.isEnabled = isPossibleSignUp
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    companion object {
        const val IMAGE_SIZE = 1024
    }
}