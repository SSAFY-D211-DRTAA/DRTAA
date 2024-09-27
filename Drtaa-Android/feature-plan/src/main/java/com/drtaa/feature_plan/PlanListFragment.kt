package com.drtaa.feature_plan

import android.view.View
import androidx.core.view.isVisible
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.navigation.fragment.navArgs
import androidx.recyclerview.widget.RecyclerView
import com.drtaa.core_map.base.BaseMapFragment
import com.drtaa.core_model.plan.Plan
import com.drtaa.core_model.util.toDate
import com.drtaa.core_ui.showSnackBar
import com.drtaa.feature_plan.adapter.PlanViewPagerAdapter
import com.drtaa.feature_plan.databinding.FragmentPlanListBinding
import com.drtaa.feature_plan.viewmodel.PlanViewModel
import com.naver.maps.map.MapView
import com.naver.maps.map.NaverMap
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber

@AndroidEntryPoint
class PlanListFragment :
    BaseMapFragment<FragmentPlanListBinding>(R.layout.fragment_plan_list) {

    private val args: PlanListFragmentArgs by navArgs()

    private val planViewModel: PlanViewModel by hiltNavGraphViewModels(R.id.nav_graph_plan)
    private lateinit var viewPagerAdapter: PlanViewPagerAdapter

    private val editPlanList = arrayListOf<ArrayList<Plan.DayPlan.PlanItem>>()

    private var fragmentList = listOf<DayPlanFragment>()

    private lateinit var datePickerDialog: DatePickerDialog

    override var mapView: MapView? = null

    override fun onResume() {
        super.onResume()

        if (planViewModel.plan.value == null) {
            return
        }

        if (!planViewModel.isViewPagerLoaded) {
            initViewPager()
        }
    }

    override fun onDestroyView() {
        super.onDestroyView()
        planViewModel.isViewPagerLoaded = false
    }

    override fun initMapView() {
        mapView = binding.mvPlanMap
        mapView?.getMapAsync(this)
    }

    override fun initOnMapReady(naverMap: NaverMap) {
        naverMap.uiSettings.isLocationButtonEnabled = false
    }

    override fun iniView() {
        initData()

        initEvent()
        initObserve()

        planViewModel.plan.value?.let {
            binding.tvPlanTitle.text = it.travelName
            binding.tvPlanDate.text =
                "${(it.travelStartDate + " ~ " + it.travelEndDate).replace('-', '.')}"
        }
    }

    private fun initData() {
        planViewModel.setTravelId(args.travelId)
        planViewModel.getPlan()
    }

    private fun initDatePickerDialog() {
        val dayIdx = binding.vpPlanDay.currentItem
        val dateArray = planViewModel.plan.value?.datesDetail?.map {
            it.travelDatesDate
        }?.toTypedArray() ?: arrayOf()

        datePickerDialog = DatePickerDialog(requireActivity(), dateArray, dayIdx)
        datePickerDialog.onCheckClickListener = object : DatePickerDialog.OnCheckClickListener {
            override fun onCheckClick(selectedDateIdx: Int) {
                // 날짜 변경 작업
                val currentDayIdx = binding.vpPlanDay.currentItem
                planViewModel.updateDate(
                    dayIdxFrom = currentDayIdx,
                    dayIdxTo = selectedDateIdx,
                    movePlanList = editPlanList[currentDayIdx]
                )
//                   //////////////////////////////////////////////// completeEdit(dayIdx)
            }
        }
    }

    private fun completeEdit() {
        editPlanList.forEachIndexed { index, _ ->
            editPlanList[index] = arrayListOf()  // 각 내부 리스트를 빈 배열로 대체
        }
        planViewModel.setEditMode(false)
    }

    private fun initObserve() {
        planViewModel.isEditMode.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isEditMode ->
                binding.tvEditPlan.isVisible = !isEditMode
                binding.tvEditFinish.isVisible = isEditMode

                if (!isEditMode) {
                    binding.clEditBottomSheet.visibility = View.GONE
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        planViewModel.plan.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { plan ->
                if (plan == null) return@onEach

                binding.tvPlanTitle.text = plan.travelName

                if (planViewModel.isViewPagerLoaded) return@onEach
                initViewPager()
                initDatePickerDialog()
                planViewModel.isViewPagerLoaded = true
                Timber.d("plan: $plan")
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        planViewModel.isEditSuccess.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isEditSuccess ->
                if (isEditSuccess == null) return@onEach

                if (isEditSuccess) {
                    completeEdit()
                } else {
                    showSnackBar("오류가 발생했습니다. 다시 시도해주세요.")
                }
                planViewModel.setIsEditSuccess(null)
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initEvent() {
        binding.ivEditTitle.setOnClickListener {
            // 여행 이름 변경 다이얼로그 띄우기
        }

        binding.tvEditPlan.setOnClickListener {
            planViewModel.setEditMode(true)
        }

        binding.tvEditFinish.setOnClickListener {
            val dayIdx = binding.vpPlanDay.currentItem
            // 순서 변경된 데이터 저장
            val newData = getRVAdapterList()
            Timber.d("dayIdx: $dayIdx, $newData")
            planViewModel.updatePlan(dayIdx, newData)

            planViewModel.setEditMode(false)
        }

        binding.btnDeletePlan.setOnClickListener {
            val dayIdx = binding.vpPlanDay.currentItem
            Timber.d("dayIdx: $dayIdx, ${editPlanList[dayIdx]}")
            planViewModel.deletePlan(dayIdx, editPlanList[dayIdx])

//                //////////////////////////////////////////////////////////////////////////////////////

        }

        binding.btnChangeDate.setOnClickListener {
            // 날짜 변경
            val dayIdx = binding.vpPlanDay.currentItem
            datePickerDialog.setDate(dayIdx)
            datePickerDialog.show()
        }

        binding.ivAddPlan.setOnClickListener {
            val dayIdx = binding.vpPlanDay.currentItem
            navigateDestination(
                PlanListFragmentDirections.actionPlanListFragmentToPlanSearchFragment(
                    day = dayIdx + 1
                )
            )
        }
    }

    private fun initViewPager() {
        if (fragmentList.isEmpty()) {
            fragmentList = planViewModel.dayPlanList.value?.let {
                List(it.size) { index ->
                    editPlanList.add(arrayListOf())

                    DayPlanFragment(
                        context = requireActivity(),
                        day = index + 1,
                        onPlanSelectListener = { planItem ->
                            if (planViewModel.isEditMode.value) {
                                editPlan(planItem)
                            }
                        }
                    )
                }
            } ?: emptyList()
        }
        viewPagerAdapter = PlanViewPagerAdapter(requireActivity(), fragmentList)

        binding.apply {
            vpPlanDay.adapter = viewPagerAdapter
            vpPlanDay.getChildAt(0).overScrollMode = RecyclerView.OVER_SCROLL_NEVER
            vpPlanDay.isUserInputEnabled = false

            updateDayPlanText(planViewModel.plan.value!!.datesDetail[0])

            ivPlanDayPrev.setOnClickListener {
                binding.vpPlanDay.apply {
                    if (currentItem > 0) {
                        currentItem -= 1

                        updateDayPlanText(planViewModel.plan.value!!.datesDetail[currentItem])
                    }
                }
            }

            ivPlanDayNext.setOnClickListener {
                binding.vpPlanDay.apply {
                    if (currentItem < fragmentList.size - 1) {
                        currentItem += 1

                        updateDayPlanText(planViewModel.plan.value!!.datesDetail[currentItem])
                    }
                }
            }
        }
    }

    private fun getRVAdapterList(): List<Plan.DayPlan.PlanItem> {
        val dayIdx = binding.vpPlanDay.currentItem
        return fragmentList[dayIdx].planListAdapter.currentList
    }

    private fun editPlan(planItem: Plan.DayPlan.PlanItem) {
        binding.clEditBottomSheet.visibility = View.VISIBLE

        val dayIdx = binding.vpPlanDay.currentItem

        if (planItem.isSelected) {
            editPlanList[dayIdx].add(planItem)
        } else {
            editPlanList[dayIdx].remove(planItem)
        }
    }

    private fun updateDayPlanText(dayPlan: Plan.DayPlan) {
        binding.tvPlanDay.text =
            "Day ${dayPlan.travelDatesId} ${dayPlan.travelDatesDate.toDate()}"
    }
}