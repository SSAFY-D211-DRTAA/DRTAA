package com.drtaa.feature_plan

import android.content.Context
import android.view.MotionEvent
import android.view.View
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.recyclerview.widget.ItemTouchHelper
import com.drtaa.core_model.plan.Plan
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_plan.adapter.ItemTouchHelperCallback
import com.drtaa.feature_plan.adapter.PlanListAdapter
import com.drtaa.feature_plan.databinding.FragmentDayPlanBinding
import com.drtaa.feature_plan.viewmodel.PlanViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber

@AndroidEntryPoint
class DayPlanFragment(
    private val context: Context,
    private val day: Int,
    private val onPlanSelectListener: (planItem: Plan.DayPlan.PlanItem) -> Unit,
) : BaseFragment<FragmentDayPlanBinding>(R.layout.fragment_day_plan) {

    private val planViewModel: PlanViewModel by hiltNavGraphViewModels(R.id.nav_graph_plan)

    val planListAdapter = PlanListAdapter(context, onPlanSelectListener)
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
                Timber.d("isEditMode : $isEditMode")
                planListAdapter.enableEditMode(isEditMode)
                helper.attachToRecyclerView(
                    if (isEditMode) binding.rvDayPlan else null
                )
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        planViewModel.dayPlanList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { dayPlanList ->
                Timber.d("dayPlanList : $dayPlanList")
                if (dayPlanList == null) return@onEach

                val dayPlan = dayPlanList[day - 1]
                if (dayPlan.placesDetail.isEmpty()) {
                    binding.llNoPlan.visibility = View.VISIBLE
                    binding.clDayPlan.visibility = View.GONE
                } else {
                    planListAdapter.submitList(dayPlan.placesDetail.toMutableList())
                    Timber.d("placesDetail : ${dayPlan.placesDetail}")
                    binding.clDayPlan.visibility = View.VISIBLE
                    binding.llNoPlan.visibility = View.GONE
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initRVAdapter() {
        binding.rvDayPlan.apply {
            itemAnimator = null
            adapter = planListAdapter
        }
    }

    private fun initEvent() {
        binding.btnAddPlan.setOnClickListener {
            navigateDestination(
                PlanListFragmentDirections.actionPlanListFragmentToPlanSearchFragment(
                    day = day
                )
            )
        }

//        binding.rvDayPlan.setOnTouchListener { view, event ->
//            when (event.action) {
//                MotionEvent.ACTION_DOWN -> {
//                    // 드래그를 시작할 때 상위 ScrollView가 터치 이벤트를 가로채지 않도록 설정
//                    view.parent.requestDisallowInterceptTouchEvent(true)
//                }
//                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
//                    // 드래그가 끝나면 원래대로 되돌리기
//                    view.parent.requestDisallowInterceptTouchEvent(false)
//                }
//            }
//            false
//        }
    }
}