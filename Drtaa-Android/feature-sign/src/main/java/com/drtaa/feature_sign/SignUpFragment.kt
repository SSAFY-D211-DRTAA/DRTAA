package com.drtaa.feature_sign

import android.content.Context
import android.net.Uri
import android.os.Environment
import android.text.Editable
import android.text.TextWatcher
import androidx.activity.result.contract.ActivityResultContracts
import androidx.fragment.app.activityViewModels
import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_sign.databinding.FragmentSignUpBinding
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import java.io.File
import java.io.FileOutputStream

@AndroidEntryPoint
class SignUpFragment : BaseFragment<FragmentSignUpBinding>(R.layout.fragment_sign_up) {

    private val signViewModel: SignViewModel by activityViewModels()
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
        binding.signUpIdEt.addTextChangedListener(object : TextWatcher {
            override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {
                signUpFragmentViewModel.setIsDuplicatedId(null)
            }

            override fun afterTextChanged(s: Editable?) {

            }

            override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {

            }
        })
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
                val buffer = ByteArray(1024)
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

                }

            }.launchIn(viewLifecycleOwner.lifecycleScope)

        signUpFragmentViewModel.isDuplicatedId.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isDuplicatedId ->
                binding.signUpIdHelpTv.text = when (isDuplicatedId) {
                    null -> ""
                    true -> "이미 사용중인 아이디입니다."
                    false -> "사용 가능한 아이디입니다."
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        signUpFragmentViewModel.profileImageUri.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { imageUri ->
                binding.signUpProfileIv.setImageURI(imageUri)
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

}