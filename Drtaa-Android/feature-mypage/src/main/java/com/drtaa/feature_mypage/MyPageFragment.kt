package com.drtaa.feature_mypage

import android.content.Context
import android.content.Intent
import android.net.Uri
import android.os.Environment
import androidx.activity.result.contract.ActivityResultContracts
import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_auth.SocialLoginManager
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.core_ui.component.TwoButtonMessageDialog
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
import javax.inject.Inject

@AndroidEntryPoint
class MyPageFragment : BaseFragment<FragmentMyPageBinding>(R.layout.fragment_my_page) {
    private val myPageViewModel: MyPageViewModel by viewModels()

    @Inject
    lateinit var socialLoginManager: SocialLoginManager

    private val getContent =
        registerForActivityResult(ActivityResultContracts.GetContent()) { uri: Uri? ->
            uri?.let {
                showLoading()
                handleImage(it)
            }
        }

    override fun initView() {
        setupMyPageItems()
        initObserve()
        initEvent()
        binding.apply {
            viewModel = this@MyPageFragment.myPageViewModel
        }
    }

    private fun initObserve() {
        myPageViewModel.logoutState.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach {
            if (it) {
                dismissLoading()
                restartApp()
            }
        }.launchIn(viewLifecycleOwner.lifecycleScope)
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
                dismissLoading()
                if (result) {
                    showSnackBar("프로필 이미지가 변경되었습니다.")
                } else {
                    showSnackBar("프로필 이미지 변경에 실패했습니다.")
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initEvent() {
        binding.btnMypageEditProfile.setOnClickListener {
            getContent.launch("image/*")
        }
    }

    private fun setupMyPageItems() {
        with(binding) {
            llMypageItem1
            llMypageItem1.apply {
                tvMypageItemTitle.text = "프로필 정보"
                imgMypageItemImage.setImageResource(R.drawable.ic_user)
                clMypageItem.setOnClickListener {
                    navigateDestination(R.id.action_myPageFragment_to_profileFragment)
                }
            }
            llMypageItem2.apply {
                tvMypageItemTitle.text = "결제 내역"
                imgMypageItemImage.setImageResource(R.drawable.ic_money)
                clMypageItem.setOnClickListener {
                    navigateDestination(R.id.action_myPageFragment_to_paymentListFragment)
                }
            }
            llMypageItem3.apply {
                tvMypageItemTitle.text = "렌트 관리"
                imgMypageItemImage.setImageResource(R.drawable.ic_car)
                clMypageItem.setOnClickListener {
                    navigateDestination(R.id.action_myPageFragment_to_rentHistoryFragment)
                }
            }
            llMypageItem4.apply {
                tvMypageItemTitle.text = "로그아웃"
                imgMypageItemImage.setImageResource(R.drawable.ic_sign_out)
                clMypageItem.setOnClickListener {
                    TwoButtonMessageDialog(
                        context = requireActivity(),
                        message = "로그아웃 하시겠습니까?",
                        onCheckClick = {
                            socialLoginManager.logout("Naver", requireActivity())
                            myPageViewModel.logout()
                            showLoading()
                        }
                    ).show()
                }
            }
        }
    }

    private fun restartApp() {
        val intent =
            requireContext().packageManager.getLaunchIntentForPackage(requireContext().packageName)
        intent?.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TASK or Intent.FLAG_ACTIVITY_NEW_TASK)

        // 현재 액티비티 종료 후 바로 새로운 액티비티 실행
        requireActivity().startActivity(intent)
        requireActivity().finish()
        Runtime.getRuntime().exit(0) // 앱 프로세스 종료
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