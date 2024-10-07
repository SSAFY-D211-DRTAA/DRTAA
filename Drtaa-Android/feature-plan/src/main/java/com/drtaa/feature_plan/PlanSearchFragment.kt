package com.drtaa.feature_plan

import android.view.KeyEvent
import android.view.View
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.fragment.app.viewModels
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.navigation.fragment.navArgs
import com.drtaa.core_map.base.BaseMapFragment
import com.drtaa.core_map.moveCameraTo
import com.drtaa.core_map.setCustomLocationButton
import com.drtaa.core_model.map.Search
import com.drtaa.core_ui.hideKeyboard
import com.drtaa.core_ui.showSnackBar
import com.drtaa.feature_plan.adapter.SearchListAdapter
import com.drtaa.feature_plan.databinding.FragmentPlanSearchBinding
import com.drtaa.feature_plan.viewmodel.PlanSearchViewModel
import com.drtaa.feature_plan.viewmodel.PlanViewModel
import com.google.android.material.bottomsheet.BottomSheetBehavior
import com.google.android.material.bottomsheet.BottomSheetBehavior.BottomSheetCallback
import com.naver.maps.map.LocationTrackingMode
import com.naver.maps.map.MapView
import com.naver.maps.map.NaverMap
import com.naver.maps.map.util.FusedLocationSource
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber

@AndroidEntryPoint
class PlanSearchFragment :
    BaseMapFragment<FragmentPlanSearchBinding>(R.layout.fragment_plan_search) {
    private val planViewModel: PlanViewModel by hiltNavGraphViewModels(R.id.nav_graph_plan)
    private val planSearchViewModel: PlanSearchViewModel by viewModels()

    private val args: PlanSearchFragmentArgs by navArgs()

    override var mapView: MapView? = null

    private lateinit var behavior: BottomSheetBehavior<ConstraintLayout>
    private lateinit var locationSource: FusedLocationSource

    private val searchListAdapter = SearchListAdapter()

    override fun initMapView() {
        mapView = binding.mvMap
        mapView?.getMapAsync(this)
    }

    override fun initOnMapReady(naverMap: NaverMap) {
        naverMap.uiSettings.isLocationButtonEnabled = false
        naverMap.setCustomLocationButton(binding.layoutPlanSearchBottomSheet.ivPlanSearchCurrentLocation)
        locationSource = FusedLocationSource(this, LOCATION_PERMISSION_REQUEST_CODE)
        naverMap.locationTrackingMode = LocationTrackingMode.Follow
        initObserve(naverMap)
    }

    override fun iniView() {
        initBottomSheet()
        initEvent()
        initObserve()
        initRV()
    }

    private fun initRV() {
        searchListAdapter.setItemClickListener(object : SearchListAdapter.ItemClickListener {
            override fun onItemClicked(search: Search) {
                planSearchViewModel.setSelectedSearchItem(search)
            }
        })

        binding.layoutPlanSearchBottomSheet.rvSearchResult.adapter = searchListAdapter
    }

    private fun initObserve(naverMap: NaverMap) {
        planSearchViewModel.selectedSearchItem.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { selectedSearchItem ->
                Timber.d("selectedSearchItem $selectedSearchItem")
                selectedSearchItem?.let {
                    naverMap.setMarker(selectedSearchItem.lat, selectedSearchItem.lng)
                    naverMap.moveCameraTo(selectedSearchItem.lat, selectedSearchItem.lng)
                    naverMap.setContentPadding(0, 0, 0, MAP_BOTTOM_CONTENT_PADDING)
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initObserve() {
        planSearchViewModel.searchList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { result ->
                result.onSuccess { data ->
                    binding.layoutPlanSearchBottomSheet.tvSearchBefore.visibility = View.GONE
                    searchListAdapter.submitList(data)
                    if (data.isEmpty()) {
                        binding.layoutPlanSearchBottomSheet.tvSearchNothing.visibility =
                            View.VISIBLE
                        binding.layoutPlanSearchBottomSheet.btnSearchSelect.visibility = View.GONE
                    } else {
                        binding.layoutPlanSearchBottomSheet.tvSearchNothing.visibility = View.GONE
                        binding.layoutPlanSearchBottomSheet.btnSearchSelect.visibility =
                            View.VISIBLE
                    }
                }.onFailure {
                    showSnackBar("오류가 발생했습니다. 다시 시도해주세요.")
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initEvent() {
        binding.layoutPlanSearchBottomSheet.apply {
            this.ivSearchLocation.setOnClickListener {
                if (this.etSearchLocation.text.isEmpty()) {
                    showSnackBar("검색어를 입력해주세요.")
                    return@setOnClickListener
                }
                planSearchViewModel.getSearchList(this.etSearchLocation.text.toString())
                requireActivity().hideKeyboard(requireView())
                behavior.state = BottomSheetBehavior.STATE_EXPANDED
            }

            this.etSearchLocation.setOnKeyListener { _, keyCode, keyEvent ->
                var handle = false
                if (keyEvent.action == KeyEvent.ACTION_DOWN && keyCode == KeyEvent.KEYCODE_ENTER) {
                    handle = true
                    this.ivSearchLocation.callOnClick()
                }

                handle
            }

            this.btnSearchSelect.setOnClickListener {
                Timber.tag("happy").d("args.day ${args.day}")
                planSearchViewModel.selectedSearchItem.value.apply {
                    if (this != null) {
                        planViewModel.addPlan(
                            dayIdx = args.day - 1,
                            newLocation = this
                        )
                        navigatePopBackStack()
                    } else {
                        showSnackBar("장소를 선택해주세요.")
                    }
                }
            }
        }
    }

    private fun initBottomSheet() {
        binding.layoutPlanSearchBottomSheet.tvSearchNothing.visibility = View.GONE
        binding.layoutPlanSearchBottomSheet.btnSearchSelect.visibility = View.GONE

        behavior = BottomSheetBehavior.from(binding.clPlanSearchBottomSheet)
        behavior.state = BottomSheetBehavior.STATE_COLLAPSED
        behavior.isHideable = false

        behavior.peekHeight = BOTTOM_SHEET_PEEK_HEIGHT
        behavior.maxHeight = BOTTOM_SHEET_PEEK_HEIGHT * 2

        behavior.addBottomSheetCallback(object : BottomSheetCallback() {
            override fun onStateChanged(bottomSheet: View, newState: Int) {
                when (newState) {
                    BottomSheetBehavior.STATE_EXPANDED -> {
                        Timber.d("STATE_EXPANDED 펼침")
                    }

                    BottomSheetBehavior.STATE_COLLAPSED -> {
                        Timber.d("STATE_COLLAPSED 접음")
                    }

                    BottomSheetBehavior.STATE_DRAGGING -> {
                        Timber.d("STATE_DRAGGING 드래그")
                    }

                    BottomSheetBehavior.STATE_SETTLING -> {
                        Timber.d("STATE_SETTLING 고정")
                    }

                    BottomSheetBehavior.STATE_HIDDEN -> {
                        Timber.d("STATE_HIDDEN 숨김")
                    }
                }
            }

            override fun onSlide(bottomSheet: View, slideOffset: Float) {
                // 슬라이드
            }
        })
    }

    companion object {
        const val BOTTOM_SHEET_PEEK_HEIGHT = 500
        const val MAP_BOTTOM_CONTENT_PADDING = 100
        const val LOCATION_PERMISSION_REQUEST_CODE = 1000
    }
}