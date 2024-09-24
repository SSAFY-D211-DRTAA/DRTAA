package com.drtaa.feature_plan

import android.view.View
import androidx.core.view.isVisible
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.recyclerview.widget.RecyclerView
import com.drtaa.core_map.base.BaseMapFragment
import com.drtaa.core_model.plan.Plan
import com.drtaa.core_model.util.toDate
import com.drtaa.feature_plan.adapter.PlanViewPagerAdapter
import com.drtaa.feature_plan.databinding.FragmentPlanListBinding
import com.drtaa.feature_plan.viewmodel.PlanViewModel
import com.drtaa.feature_plan.viewmodel.PlanViewModel.Companion.seoulTrip
import com.naver.maps.map.MapView
import com.naver.maps.map.NaverMap
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach

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
        naverMap.uiSettings.isLocationButtonEnabled = false
    }

    override fun iniView() {
        initViewPager()
        initEvent()
        initObserve()
    }

    private fun initObserve() {
        planViewModel.isEditMode.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isEditMode ->
                binding.tvEditPlan.isVisible = !isEditMode
                binding.tvEditFinish.isVisible = isEditMode
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initEvent() {
        binding.tvEditPlan.setOnClickListener {
            planViewModel.setEditMode(true)
        }

        binding.tvEditFinish.setOnClickListener {
            planViewModel.setEditMode(false)
        }
    }

    private fun initViewPager() {
        val fragmentList = planViewModel.plan.value?.let {
            it.datesDetail.map { dayPlan ->
                DayPlanFragment(dayPlan.travelDatesId)
            }
        } ?: emptyList()
        viewPagerAdapter = PlanViewPagerAdapter(requireActivity(), fragmentList)

        binding.apply {
            vpPlanDay.adapter = viewPagerAdapter
            vpPlanDay.getChildAt(0).overScrollMode = RecyclerView.OVER_SCROLL_NEVER
            vpPlanDay.isUserInputEnabled = false

            updateDayPlanText(seoulTrip.datesDetail[0])

            ivPlanDayPrev.setOnClickListener {
                binding.vpPlanDay.apply {
                    if (currentItem > 0) {
                        currentItem -= 1

                        updateDayPlanText(seoulTrip.datesDetail[currentItem])
                    }
                }
            }

            ivPlanDayNext.setOnClickListener {
                binding.vpPlanDay.apply {
                    if (currentItem < fragmentList.size - 1) {
                        currentItem += 1

                        updateDayPlanText(seoulTrip.datesDetail[currentItem])
                    }
                }
            }
        }
    }

    private fun updateDayPlanText(dayPlan: Plan.DayPlan) {
        binding.tvPlanDay.text =
            "Day ${dayPlan.travelDatesId} ${dayPlan.travelDatesDate.toDate()}"
    }
}