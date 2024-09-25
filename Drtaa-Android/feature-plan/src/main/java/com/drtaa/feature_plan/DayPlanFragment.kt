package com.drtaa.feature_plan

import android.content.Context
import android.view.View
import androidx.fragment.app.viewModels
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.recyclerview.widget.ItemTouchHelper
import com.drtaa.core_model.plan.Plan
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_plan.adapter.ItemTouchHelperCallback
import com.drtaa.feature_plan.adapter.PlanListAdapter
import com.drtaa.feature_plan.databinding.FragmentDayPlanBinding
import com.drtaa.feature_plan.viewmodel.DayViewModel
import com.drtaa.feature_plan.viewmodel.PlanViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach

@AndroidEntryPoint
class DayPlanFragment(
    private val context: Context,
    private val day: Int,
    private val onPlanSelectListener: (planItem: Plan.DayPlan.PlanItem) -> Unit,
) : BaseFragment<FragmentDayPlanBinding>(R.layout.fragment_day_plan) {

    private val planViewModel: PlanViewModel by hiltNavGraphViewModels(R.id.nav_graph_plan)

    private val planListAdapter = PlanListAdapter(context, onPlanSelectListener)
    private val itemTouchHelperCallback = ItemTouchHelperCallback(planListAdapter)
    private val helper = ItemTouchHelper(itemTouchHelperCallback)

    override fun initView() {
        initObserve()
        initEvent()
        initRVAdapter()
    }

    private fun initObserve() {
        planViewModel.isEditMode.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isEditMode ->
                planListAdapter.enableEditMode(isEditMode)
                helper.attachToRecyclerView(
                    if (isEditMode) binding.rvDayPlan else null
                )
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        planViewModel.dayPlanList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { dayPlanList ->
                if (dayPlanList == null) return@onEach

                val dayPlan = dayPlanList[day-1]
                if (dayPlan.placesDetail.isEmpty()) {
                    binding.llNoPlan.visibility = View.VISIBLE
                } else {
                    binding.clDayPlan.visibility = View.VISIBLE
                    planListAdapter.submitList(dayPlan.placesDetail)
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initRVAdapter() {
        binding.rvDayPlan.adapter = planListAdapter
    }

    private fun initEvent() {
        binding.btnAddPlan.setOnClickListener {
            navigateDestination(R.id.action_planListFragment_to_planSearchFragment)
        }
    }
}