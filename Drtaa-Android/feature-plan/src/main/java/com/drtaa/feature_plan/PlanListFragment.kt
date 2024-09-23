package com.drtaa.feature_plan

import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.recyclerview.widget.RecyclerView
import com.drtaa.core_map.base.BaseMapFragment
import com.drtaa.feature_plan.adapter.PlanViewPagerAdapter
import com.drtaa.feature_plan.databinding.FragmentPlanListBinding
import com.drtaa.feature_plan.viewmodel.PlanViewModel
import com.naver.maps.map.MapView
import com.naver.maps.map.NaverMap
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class PlanListFragment :
    BaseMapFragment<FragmentPlanListBinding>(R.layout.fragment_plan_list) {

    private val planViewModel: PlanViewModel by hiltNavGraphViewModels(R.id.nav_graph_plan)
    private lateinit var viewPagerAdapter: PlanViewPagerAdapter

    override var mapView: MapView? = null

    override fun initMapView() {
        mapView = binding.mvPlanMap
        mapView?.getMapAsync(this)
    }

    override fun initOnMapReady(naverMap: NaverMap) {

    }

    override fun iniView() {
        initViewPager()
    }

    private fun initViewPager() {
        val fragmentList = listOf(DayPlanFragment(1), DayPlanFragment(2))
        viewPagerAdapter = PlanViewPagerAdapter(requireActivity(), fragmentList)

        binding.apply {
            vpPlanDay.adapter = viewPagerAdapter
            vpPlanDay.getChildAt(0).overScrollMode = RecyclerView.OVER_SCROLL_NEVER

            ivPlanDayPrev.setOnClickListener {
                binding.vpPlanDay.apply {
                    if (currentItem > 0) {
                        currentItem -= 1
                        tvPlanDay.text = "Day ${currentItem + 1}"
                    }
                }
            }

            ivPlanDayNext.setOnClickListener {
                binding.vpPlanDay.apply {
                    if (currentItem < fragmentList.size - 1) {
                        currentItem += 1
                        tvPlanDay.text = "Day ${currentItem + 1}"
                    }
                }
            }
        }
    }
}