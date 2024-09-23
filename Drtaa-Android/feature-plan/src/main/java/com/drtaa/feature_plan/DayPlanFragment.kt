package com.drtaa.feature_plan

import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_plan.databinding.FragmentDayPlanBinding
import com.drtaa.feature_plan.viewmodel.PlanViewModel
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class DayPlanFragment(
    private val day: Int
) : BaseFragment<FragmentDayPlanBinding>(R.layout.fragment_day_plan) {

    private val planViewModel: PlanViewModel by hiltNavGraphViewModels(R.id.nav_graph_plan)

    override fun initView() {
        binding.tvDay.text = "Day $day"
    }
}