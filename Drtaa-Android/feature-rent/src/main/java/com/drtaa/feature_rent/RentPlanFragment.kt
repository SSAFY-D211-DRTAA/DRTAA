package com.drtaa.feature_rent

import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.feature_rent.databinding.FragmentRentPlanBinding
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class RentPlanFragment : BaseFragment<FragmentRentPlanBinding>(R.layout.fragment_rent_plan) {

//    private val rentViewModel: RentViewModel by hiltNavGraphViewModels(R.id.nav_graph_rent)

    override fun initView() {
        initEvent()
        initObserve()
    }

    private fun initObserve() {
        //
    }

    private fun initEvent() {
        //
    }
}
