package com.drtaa.feature_plan

import android.view.View
import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_ui.base.BaseFragment
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

    private val planHistoryListAdapter = PlanHistoryListAdapter()

    override fun initView() {
        initObserve()
        initRVAdapter()
    }

    private fun initRVAdapter() {
        planHistoryListAdapter.setItemClickListener(object :
            PlanHistoryListAdapter.ItemClickListener {
            override fun onItemClicked(travelId: Int) {
                navigateDestination(
                    PlanHistoryFragmentDirections.actionPlanHistoryFragmentToPlanListFragment(
                        travelId = travelId
                    )
                )
            }
        })
        binding.rvPlanHistory.adapter = planHistoryListAdapter
    }

    private fun initObserve() {
        planHistoryViewModel.planList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { planList ->
                if (planList == null) return@onEach

                if (planList.isEmpty()) {
                    binding.tvNoPlanHistory.visibility = View.VISIBLE
                    binding.rvPlanHistory.visibility = View.GONE
                } else {
                    binding.tvNoPlanHistory.visibility = View.GONE
                    binding.rvPlanHistory.visibility = View.VISIBLE

                    planHistoryListAdapter.submitList(planList)
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }
}