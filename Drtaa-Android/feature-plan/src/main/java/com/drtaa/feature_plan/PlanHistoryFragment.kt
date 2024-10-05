package com.drtaa.feature_plan

import android.net.Uri
import android.view.View
import androidx.activity.addCallback
import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.navigation.NavDeepLinkRequest
import androidx.navigation.NavOptions
import androidx.navigation.fragment.findNavController
import com.drtaa.core_ui.DeepLinkConstants
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_plan.adapter.PlanHistoryListAdapter
import com.drtaa.feature_plan.databinding.FragmentPlanHistoryBinding
import com.drtaa.feature_plan.viewmodel.PlanHistoryViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber

@AndroidEntryPoint
class PlanHistoryFragment :
    BaseFragment<FragmentPlanHistoryBinding>(R.layout.fragment_plan_history) {

    private val planHistoryViewModel: PlanHistoryViewModel by viewModels()

    private val planHistoryListAdapter = PlanHistoryListAdapter()

    override fun onResume() {
        super.onResume()
        planHistoryViewModel.getPlanList()
    }

    override fun initView() {
        initObserve()
        initRVAdapter()
        setupBackPressHandler()
    }

    private fun initRVAdapter() {
        planHistoryListAdapter.setItemClickListener(object :
            PlanHistoryListAdapter.ItemClickListener {
            override fun onItemClicked(travelId: Int, rentId: Int) {
                navigateDestination(
                    PlanHistoryFragmentDirections.actionPlanHistoryFragmentToPlanListFragment(
                        travelId = travelId,
                        rentId = rentId
                    )
                )
            }
        })
        binding.rvPlanHistory.adapter = planHistoryListAdapter
        binding.rvPlanHistory.itemAnimator = null
    }

    private fun initObserve() {
        planHistoryViewModel.planList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { planList ->
                Timber.tag("search").d("success $planList")
                if (planList == null) {
                    binding.tvErrorPlanHistory.visibility = View.VISIBLE
                    binding.tvNoPlanHistory.visibility = View.GONE
                    binding.rvPlanHistory.visibility = View.GONE
                } else if (planList.isEmpty()) {
                    binding.tvErrorPlanHistory.visibility = View.GONE
                    binding.tvNoPlanHistory.visibility = View.VISIBLE
                    binding.rvPlanHistory.visibility = View.GONE
                } else {
                    binding.tvErrorPlanHistory.visibility = View.GONE
                    binding.tvNoPlanHistory.visibility = View.GONE
                    binding.rvPlanHistory.visibility = View.VISIBLE

                    planHistoryListAdapter.submitList(planList)
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun setupBackPressHandler() {
        requireActivity().onBackPressedDispatcher.addCallback(viewLifecycleOwner) {
            navigateToHome()
        }
    }

    private fun navigateToHome() {
        val request = NavDeepLinkRequest.Builder
            .fromUri(Uri.parse(DeepLinkConstants.HOME))
            .build()
        findNavController().navigate(
            request,
            NavOptions.Builder()
                .setPopUpTo(findNavController().graph.startDestinationId, true)
                .build()
        )
    }
}