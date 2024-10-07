package com.drtaa.feature_plan

import android.net.Uri
import android.os.Build
import android.view.View
import androidx.activity.addCallback
import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.navigation.NavDeepLinkRequest
import androidx.navigation.NavOptions
import androidx.navigation.fragment.findNavController
import com.drtaa.core_model.map.Search
import com.drtaa.core_ui.DeepLinkConstants
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.core_ui.component.TwoButtonMessageDialog
import com.drtaa.core_ui.expandLayout
import com.drtaa.core_ui.foldLayout
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
    private val planReservedListAdapter = PlanHistoryListAdapter()
    private val planCompletedListAdapter = PlanHistoryListAdapter()
    private var isDialogShown = false

    override fun onResume() {
        super.onResume()
        planHistoryViewModel.getPlanList()
        if (!isDialogShown) {
            val searchRequest = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                arguments?.getParcelable<Search>("recommend", Search::class.java)
            } else {
                arguments?.getParcelable<Search>("recommend")
            }

            searchRequest?.let { recommend ->
                val dialog = TwoButtonMessageDialog(
                    requireContext(),
                    "추천받은 장소를 일정에 추가할까요? 일정 마지막에 추가되며, " +
                            "필요시 순서를 바꿔주세요!" +
                            "\n${recommend.title}\n${recommend.roadAddress}"
                ) {
                    planHistoryViewModel.planInProgress.value?.let { progress ->
                        moveToPlanListFragment(progress.travelId, progress.rentId, recommend)
                    }
                }

                dialog.show()
                isDialogShown = true
                Timber.tag("plan").d("$recommend")
            }
        }
    }

    override fun initView() {
        showLoading()
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

            cvPlanInProgress.setOnClickListener {
                planHistoryViewModel.planInProgress.value?.let {
                    moveToPlanListFragment(it.travelId, it.rentId)
                }
            }
        }
    }

    private fun initRVAdapter() {
        val clickListener = object :
            PlanHistoryListAdapter.ItemClickListener {
            override fun onItemClicked(travelId: Int, rentId: Int) {
                moveToPlanListFragment(travelId, rentId)
            }
        }
        planReservedListAdapter.setItemClickListener(clickListener)
        planCompletedListAdapter.setItemClickListener(clickListener)

        binding.rvPlanReservedHistory.adapter = planReservedListAdapter
        binding.rvPlanCompletedHistory.adapter = planCompletedListAdapter

        binding.rvPlanReservedHistory.itemAnimator = null
        binding.rvPlanCompletedHistory.itemAnimator = null
    }

    private fun moveToPlanListFragment(travelId: Int, rentId: Int, recommend: Search? = null) {
        navigateDestination(
            PlanHistoryFragmentDirections.actionPlanHistoryFragmentToPlanListFragment(
                travelId = travelId,
                rentId = rentId,
                recommend = recommend
            )
        )
    }

    private fun initObserve() {
        planHistoryViewModel.planList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { planList ->
                if (planList == null) {
                    binding.tvErrorPlanHistory.visibility = View.VISIBLE
                    binding.clPlan.visibility = View.GONE
                } else {
                    dismissLoading()
                    binding.tvErrorPlanHistory.visibility = View.GONE
                    binding.clPlan.visibility = View.VISIBLE
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        planHistoryViewModel.planInProgress.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { planInProgress ->
                if (planInProgress == null) {
                    binding.cvPlanInProgress.visibility = View.GONE
                    binding.clPlanNoInProgress.visibility = View.VISIBLE
                } else {
                    dismissLoading()
                    binding.cvPlanInProgress.visibility = View.VISIBLE
                    binding.clPlanNoInProgress.visibility = View.GONE

                    binding.planSimple = planInProgress
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