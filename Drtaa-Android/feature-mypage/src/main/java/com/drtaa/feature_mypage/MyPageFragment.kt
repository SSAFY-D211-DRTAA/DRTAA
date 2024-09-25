package com.drtaa.feature_mypage

import android.content.Context
import android.net.Uri
import android.os.Environment
import androidx.activity.result.contract.ActivityResultContracts
import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.core_ui.showSnackBar
import com.drtaa.feature_mypage.databinding.FragmentMyPageBinding
import com.drtaa.feature_mypage.viewmodel.MyPageViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber
import java.io.File
import java.io.FileOutputStream
import java.security.SecureRandom

@AndroidEntryPoint
class MyPageFragment : BaseFragment<FragmentMyPageBinding>(R.layout.fragment_my_page) {
    private val myPageViewModel: MyPageViewModel by viewModels()

    private val getContent = registerForActivityResult(ActivityResultContracts.GetContent()) { uri: Uri? ->
        uri?.let {
            handleImage(it)
        }
    }

    override fun initView() {
        initObserve()
        initEvent()
        setupMyPageItems()
        binding.apply {
            viewModel = this@MyPageFragment.myPageViewModel
        }
    }

    private fun initObserve() {
        myPageViewModel.currentUser.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { result ->
                if (result == null) return@onEach
                binding.socialUser = result
                Timber.d("$result")
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        myPageViewModel.profileImageUri.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { imageUri ->
                binding.imgMypageProfile.setImageURI(imageUri)
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        myPageViewModel.updateResult.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { result ->
                if (result) showSnackBar("프로필 이미지가 변경되었습니다.")
                else showSnackBar("프로필 이미지 변경에 실패했습니다.")
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initEvent() {
        binding.btnMypageEditProfile.setOnClickListener {
            getContent.launch("image/*")
        }
    }

    private fun setupMyPageItems() {
        with(binding) {
            llMypageItem1.setOnClickListener {
                showSnackBar("1클릭")
            }
            llMypageItem2.setOnClickListener {
                showSnackBar("2클릭")
            }
            llMypageItem3.setOnClickListener {
                showSnackBar("3클릭")
            }
        }
    }

    private fun handleImage(imageUri: Uri) {
        val imageFile = uriToFile(requireActivity(), imageUri)
        myPageViewModel.setProfileImage(imageUri, imageFile)
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

    companion object {
        const val IMAGE_SIZE = 1024
    }
}