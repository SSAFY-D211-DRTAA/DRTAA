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
import com.drtaa.core_ui.expandLayout
import com.drtaa.core_ui.foldLayout
import com.drtaa.feature_plan.adapter.PlanHistoryListAdapter
import com.drtaa.feature_plan.databinding.FragmentPlanHistoryBinding
import com.drtaa.feature_plan.viewmodel.PlanHistoryViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach

@AndroidEntryPoint
class PlanHistoryFragment :
    BaseFragment<FragmentPlanHistoryBinding>(R.layout.fragment_plan_history) {

    private val planHistoryViewModel: PlanHistoryViewModel by viewModels()

    private val planReservedListAdapter = PlanHistoryListAdapter()
    private val planCompletedListAdapter = PlanHistoryListAdapter()

    override fun onResume() {
        super.onResume()
        planHistoryViewModel.getPlanList()
    }

    override fun initView() {
        initRVAdapter()
        initObserve()
        setupBackPressHandler()
        initEvent()
    }

    private fun initEvent() {
        binding.apply {
            llPlanReserved.setOnClickListener {
                if (clPlanReserved.visibility == View.GONE) {
                    expandLayout(ivReservedExpand, clPlanReserved)
                } else {
                    foldLayout(ivReservedExpand, clPlanReserved)
                }
            }

            llPlanCompleted.setOnClickListener {
                if (clPlanCompleted.visibility == View.GONE) {
                    expandLayout(ivCompletedExpand, clPlanCompleted)
                } else {
                    foldLayout(ivCompletedExpand, clPlanCompleted)
                }
            }
        }
    }

    private fun initRVAdapter() {
        val clickListener = object :
            PlanHistoryListAdapter.ItemClickListener {
            override fun onItemClicked(travelId: Int, rentId: Int) {
                navigateDestination(
                    PlanHistoryFragmentDirections.actionPlanHistoryFragmentToPlanListFragment(
                        travelId = travelId,
                        rentId = rentId
                    )
                )
            }
        }
        planReservedListAdapter.setItemClickListener(clickListener)
        planCompletedListAdapter.setItemClickListener(clickListener)

        binding.rvPlanReservedHistory.adapter = planReservedListAdapter
        binding.rvPlanCompletedHistory.adapter = planCompletedListAdapter

        binding.rvPlanReservedHistory.itemAnimator = null
        binding.rvPlanCompletedHistory.itemAnimator = null
    }

    private fun initObserve() {
        planHistoryViewModel.planList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { planList ->
                if (planList == null) {
                    binding.tvErrorPlanHistory.visibility = View.VISIBLE
                    binding.clPlan.visibility = View.GONE
                } else {
                    binding.tvErrorPlanHistory.visibility = View.GONE
                    binding.clPlan.visibility = View.VISIBLE
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        planHistoryViewModel.planInProgress.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { planInProgress ->
                if (planInProgress == null) {
                    binding.clPlanNoInProgress.visibility = View.VISIBLE
                } else {
                    binding.clPlanNoInProgress.visibility = View.GONE
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        planHistoryViewModel.planReservedList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { reservedList ->
                if (reservedList == null) return@onEach

                if (reservedList.isEmpty()) {
                    binding.clPlanNoReserved.visibility = View.VISIBLE
                    binding.rvPlanReservedHistory.visibility = View.GONE
                } else {
                    binding.clPlanNoReserved.visibility = View.GONE
                    binding.rvPlanReservedHistory.visibility = View.VISIBLE

                    planReservedListAdapter.submitList(reservedList)
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        planHistoryViewModel.planCompletedList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { completedList ->
                if (completedList == null) return@onEach

                if (completedList.isEmpty()) {
                    binding.clPlanNoCompleted.visibility = View.VISIBLE
                    binding.rvPlanCompletedHistory.visibility = View.GONE
                } else {
                    binding.clPlanNoCompleted.visibility = View.GONE
                    binding.rvPlanCompletedHistory.visibility = View.VISIBLE

                    planCompletedListAdapter.submitList(completedList)
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