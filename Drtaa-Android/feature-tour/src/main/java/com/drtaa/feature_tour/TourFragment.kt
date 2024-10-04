package com.drtaa.feature_tour

import android.view.View
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.paging.LoadState
import com.drtaa.core_model.plan.PlanItem
import com.drtaa.core_model.util.toPlanItem
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.core_ui.component.LocationHelper
import com.drtaa.core_ui.dpToPx
import com.drtaa.feature_plan.adapter.PlanListAdapter
import com.drtaa.feature_tour.component.TourAdapter
import com.drtaa.feature_tour.databinding.FragmentTourBinding
import com.drtaa.feature_tour.viewmodel.TourViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

@AndroidEntryPoint
class TourFragment : BaseFragment<FragmentTourBinding>(R.layout.fragment_tour) {

    @Inject
    lateinit var locationHelper: LocationHelper
    private val tourViewModel: TourViewModel by hiltNavGraphViewModels(R.id.nav_graph_tour)

    private val tourAdapter by lazy {
        TourAdapter(onTourClickListener = { tourItem ->
            moveToTravel(tourItem.toPlanItem())
        })
    }

    private val planAdapter by lazy {
        PlanListAdapter(
            context = requireActivity(),
            onPlanSelectListener = { },
            onPlanClickListener = { planItem ->
                moveToTravel(planItem)
            }
        )
    }

    override fun onResume() {
        super.onResume()
        Timber.d("isExpanded: ${tourViewModel.isExpanded}")
        if (tourViewModel.isExpanded) {
            expandPlan()
        }
    }

    override fun initView() {
        initUI()
        getLocation()
        initObserve()
        initEvent()
    }

    private fun initEvent() {
        binding.apply {
            llExpandPlan.setOnClickListener {
                if (clPlan.visibility == View.GONE) {
                    expandPlan()
                } else {
                    foldPlan()
                }
            }
        }
    }

    private fun foldPlan() {
        Timber.d("isExpanded: 접힘")
        tourViewModel.setExpandedPlan(false)
        binding.apply {
            clPlan.visibility = View.GONE
            ivExpendPlan.animate().apply {
                duration = DURATION
                rotation(ROTATION_90)
            }
        }
    }

    private fun expandPlan() {
        Timber.d("isExpanded: 펼침")
        tourViewModel.setExpandedPlan(true)
        binding.apply {
            clPlan.visibility = View.VISIBLE
            ivExpendPlan.animate().apply {
                duration = DURATION
                rotation(ROTATION_270)
            }
        }
    }

    private fun initUI() {
        binding.rvTour.adapter = tourAdapter
        binding.rvPlan.adapter = planAdapter

        setRVMaxHeight()
    }

    private fun setRVMaxHeight() {
        // RecyclerView의 maxHeight를 250dp로 설정
        binding.apply {
            rvPlan.viewTreeObserver.addOnGlobalLayoutListener {
                val maxHeightInPx = MAX_HEIGHT.dpToPx()
                if (rvPlan.height > maxHeightInPx) {
                    val layoutParams = rvPlan.layoutParams
                    layoutParams.height = maxHeightInPx
                    rvPlan.layoutParams = layoutParams
                }
            }
        }
    }

    private fun getLocation() {
        viewLifecycleOwner.lifecycleScope.launch {
            locationHelper.getLastLocation().let { location ->
                location?.let {
                    tourViewModel.getLocationBasedList(
                        location.longitude.toString(),
                        location.latitude.toString(),
                        "1000" // 반경 2키로 검색
                    )
                }
            }
        }
    }

    private fun moveToTravel(planItem: PlanItem) {
        navigateDestination(
            TourFragmentDirections.actionFragmentTourToNavGraphTravel(
                planItem = planItem
            )
        )
    }

    private fun initObserve() {
        tourViewModel.planList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { planList ->
                binding.apply {
                    if (planList == null) {
                        rvPlan.visibility = View.GONE
                        clPlanEmpty.visibility = View.VISIBLE
                        tvPlanHelp.text = "오류가 발생했습니다.\n다시 시도해주세요."
                    } else if (planList.isEmpty()) {
                        rvPlan.visibility = View.GONE
                        clPlanEmpty.visibility = View.VISIBLE
                        tvPlanHelp.text = "일정이 비어있어요!"
                    } else {
                        planAdapter.submitList(planList)
                        rvPlan.visibility = View.VISIBLE
                        clPlanEmpty.visibility = View.GONE
                    }
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        tourViewModel.pagedTour.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { pagingData ->
                pagingData.let { tourData ->
                    Timber.d("$tourData")
                    tourAdapter.submitData(tourData)
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        tourAdapter.loadStateFlow.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { loadStates ->
                val isLoading =
                    loadStates.source.refresh is LoadState.Loading || loadStates.source.append is LoadState.Loading
                if (!isLoading) dismissLoading() else showLoading()
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    companion object {
        const val MAX_HEIGHT = 250
        const val DURATION = 300L
        const val ROTATION_90 = 90f
        const val ROTATION_270 = 270f
    }
}