package com.drtaa.feature_plan

import android.view.Gravity
import android.view.View
import androidx.core.view.isVisible
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.navigation.fragment.navArgs
import androidx.recyclerview.widget.RecyclerView
import com.drtaa.core_map.base.BaseMapFragment
import com.drtaa.core_map.setCustomLocationButton
import com.drtaa.core_model.plan.DayPlan
import com.drtaa.core_model.plan.PlanItem
import com.drtaa.core_model.util.toDate
import com.drtaa.core_ui.component.OneButtonMessageDialog
import com.drtaa.core_ui.component.TwoButtonMessageDialog
import com.drtaa.core_ui.component.TwoButtonTypingDialog
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
import java.time.LocalDate

@AndroidEntryPoint
class PlanListFragment :
    BaseMapFragment<FragmentPlanListBinding>(R.layout.fragment_plan_list) {

    private val args: PlanListFragmentArgs by navArgs()

    private val planViewModel: PlanViewModel by hiltNavGraphViewModels(R.id.nav_graph_plan)
    private lateinit var viewPagerAdapter: PlanViewPagerAdapter

    private val editPlanList = arrayListOf<ArrayList<PlanItem>>()

    private var fragmentList = listOf<DayPlanFragment>()

    private lateinit var datePickerDialog: DatePickerDialog

    override var mapView: MapView? = null

    override fun onResume() {
        super.onResume()

        if (planViewModel.isNewPlanPage) {
            initData()
            planViewModel.isNewPlanPage = false
        }

        planViewModel.plan.value?.let {
            if (!planViewModel.isViewPagerLoaded) {
                initViewPager()
            }
        }
    }

    override fun onDestroyView() {
        super.onDestroyView()
        planViewModel.setEditMode(false)
        planViewModel.isViewPagerLoaded = false
        planViewModel.isNewPlanPage = true
    }

    override fun initMapView() {
        mapView = binding.mvPlanMap
        mapView?.getMapAsync(this)
    }

    override fun initOnMapReady(naverMap: NaverMap) {
        naverMap.apply {
            minZoom = MIN_ZOOM
            uiSettings.isLocationButtonEnabled = false
            setCustomLocationButton(binding.ivPlanCurrentLocation)
            uiSettings.logoGravity = Gravity.END
            uiSettings.setLogoMargin(0, NAVER_LOGO_MARGIN, NAVER_LOGO_MARGIN, 0)
        }

        setMapMarkers(naverMap)
    }

    override fun iniView() {
        initEvent()
        initObserve()
        initEditObserve()

        planViewModel.plan.value?.let {
            binding.tvPlanTitle.text = it.travelName
            binding.tvPlanDate.text =
                "${(it.travelStartDate + " ~ " + it.travelEndDate).replace('-', '.')}"
        }
    }

    private fun initData() {
        planViewModel.setInfo(args.travelId, args.rentId)
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
            }
        }
    }

    private fun completeEdit() {
        editPlanList.forEachIndexed { index, _ ->
            editPlanList[index] = arrayListOf()
        }
        planViewModel.setEditMode(false)
    }

    private fun initObserve() {
        planViewModel.plan.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { plan ->
                if (plan == null) return@onEach

                binding.tvPlanTitle.text = plan.travelName
                binding.tvPlanDate.text =
                    "${(plan.travelStartDate + " ~ " + plan.travelEndDate).replace('-', '.')}"
                if (planViewModel.isViewPagerLoaded) return@onEach
                initViewPager()
                initDatePickerDialog()
                planViewModel.isViewPagerLoaded = true
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

    private fun initEditObserve() {
        planViewModel.isEditMode.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isEditMode ->
                binding.tvEditPlan.isVisible = !isEditMode
                binding.tvEditFinish.isVisible = isEditMode

                if (!isEditMode) {
                    binding.clEditBottomSheet.visibility = View.GONE
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        planViewModel.dayPlan.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { dayPlan ->
                if (dayPlan == null) return@onEach

                updateDayPlanText(dayPlan)

                val targetDate = LocalDate.parse(dayPlan.travelDatesDate)
                val today = LocalDate.now()
                if ((today > targetDate) or dayPlan.travelDatesIsExpired) {
                    setEditable(false)
                } else {
                    setEditable(true)
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun setEditable(isEditable: Boolean) {
        binding.apply {
            if (isEditable) {
                tvEditPlan.isVisible = !planViewModel.isEditMode.value
                tvEditFinish.isVisible = planViewModel.isEditMode.value
                ivAddPlan.isVisible = true
            } else {
                tvEditPlan.isVisible = false
                tvEditFinish.isVisible = false
                ivAddPlan.isVisible = false
            }
        }
    }

    private fun initEvent() {
        binding.ivEditTitle.setOnClickListener {
            TwoButtonTypingDialog(
                context = requireActivity(),
                defaultText = binding.tvPlanTitle.text.toString(),
                onCheckClick = { newText ->
                    binding.tvPlanTitle.text = newText
                    planViewModel.updatePlanName(newText)
                }
            ).show()
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
            if (editPlanList[binding.vpPlanDay.currentItem].isEmpty()) {
                showSnackBar("삭제할 일정이 없습니다.")
                return@setOnClickListener
            }

            val afterDeleteListSize = planViewModel.dayPlan.value?.let { dayPlan ->
                editPlanList[binding.vpPlanDay.currentItem].size - dayPlan.placesDetail.size
            }
            if (afterDeleteListSize == 0) {
                OneButtonMessageDialog(
                    context = requireActivity(),
                    message = "일정은 최소 하나 이상 남아있어야 합니다.",
                ).show()
                return@setOnClickListener
            }

            TwoButtonMessageDialog(
                context = requireActivity(),
                message = "일정을 삭제하시겠습니까?",
                onCheckClick = {
                    val dayIdx = binding.vpPlanDay.currentItem
                    Timber.d("dayIdx: $dayIdx, ${editPlanList[dayIdx]}")
                    planViewModel.deletePlan(dayIdx, editPlanList[dayIdx])
                }
            ).show()
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

    private fun setMapMarkers(naverMap: NaverMap) {
        planViewModel.dayPlan.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { dayPlan ->
                if (dayPlan == null) return@onEach

                naverMap.clearMarkerList()

                dayPlan.placesDetail.forEachIndexed { index, place ->
                    naverMap.addMarker(
                        place.datePlacesLat,
                        place.datePlacesLon,
                        index + 1,
                        place.datePlacesName
                    )
                }

                naverMap.adjustCamera()
            }.launchIn(viewLifecycleOwner.lifecycleScope)
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
                        },
                        onPlanClickListener = { planItem ->
                            moveToTravel(planItem)
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
                changeDay(-1)
            }

            ivPlanDayNext.setOnClickListener {
                changeDay(1)
            }
        }
    }

    private fun moveToTravel(planItem: PlanItem) {
        navigateDestination(
            PlanListFragmentDirections.actionFragmentPlanToNavGraphTravel(
                planItem = planItem
            )
        )
    }

    private fun getRVAdapterList(): List<PlanItem> {
        val dayIdx = binding.vpPlanDay.currentItem
        return fragmentList[dayIdx].planListAdapter.currentList
    }

    private fun editPlan(planItem: PlanItem) {
        val dayIdx = binding.vpPlanDay.currentItem

        binding.clEditBottomSheet.visibility = View.VISIBLE

        if (planItem.isSelected) {
            editPlanList[dayIdx].add(planItem)
        } else {
            editPlanList[dayIdx].remove(planItem)
            if (editPlanList[dayIdx].isEmpty()) {
                binding.clEditBottomSheet.visibility = View.GONE
            }
        }
    }

    private fun changeDay(direction: Int) {
        binding.vpPlanDay.apply {
            val newIndex = currentItem + direction
            if (newIndex in fragmentList.indices) {
                currentItem = newIndex
                updateDayPlanText(planViewModel.plan.value!!.datesDetail[newIndex])
                updateBottomSheetVisibility(newIndex)
            }
            planViewModel.setDayIdx(currentItem)
        }
    }

    private fun updateBottomSheetVisibility(dayIdx: Int) {
        if (planViewModel.isEditMode.value) {
            binding.clEditBottomSheet.visibility =
                if (editPlanList[dayIdx].isEmpty()) View.GONE else View.VISIBLE
        } else {
            binding.clEditBottomSheet.visibility = View.GONE
        }
    }

    private fun updateDayPlanText(dayPlan: DayPlan) {
        binding.tvPlanDay.text =
            "Day ${binding.vpPlanDay.currentItem + 1} ${dayPlan.travelDatesDate.toDate()}"
    }

    companion object {
        const val MIN_ZOOM = 5.0
        const val NAVER_LOGO_MARGIN = 10
    }
}