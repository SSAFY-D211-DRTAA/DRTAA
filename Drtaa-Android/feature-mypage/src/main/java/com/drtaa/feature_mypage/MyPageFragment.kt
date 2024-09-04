package com.drtaa.feature_mypage

import android.os.Bundle
import androidx.fragment.app.viewModels
import com.drtaa.core_ui.base.BaseDialogFragment
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_mypage.databinding.FragmentMyPageBinding
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MyPageFragment : BaseDialogFragment<FragmentMyPageBinding>(R.layout.fragment_my_page) {
    private val viewModel: MyPageViewModel by viewModels()

    override fun initView(savedInstanceState: Bundle?) {
        binding.apply {
            binding.viewModel = this@MyPageFragment.viewModel
        }
    }
}