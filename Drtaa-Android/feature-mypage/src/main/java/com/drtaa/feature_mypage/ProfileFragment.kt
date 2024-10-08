package com.drtaa.feature_mypage

import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_mypage.databinding.FragmentProfileBinding
import com.drtaa.feature_mypage.viewmodel.ProfileViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber

@AndroidEntryPoint
class ProfileFragment : BaseFragment<FragmentProfileBinding>(R.layout.fragment_profile) {
    private val profileViewModel: ProfileViewModel by viewModels()

    override fun initView() {
        initObserver()
    }

    override fun onResume() {
        super.onResume()
        profileViewModel.getUserData()
    }

    private fun initObserver() {
        profileViewModel.currentUser.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { result ->
                if (result == null) return@onEach
                binding.profile = result
                Timber.d("지금 현재 유저는?? $result")
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }
}