package com.drtaa.feature_rent

import android.view.KeyEvent
import android.view.View
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.fragment.app.viewModels
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_map.base.BaseMapFragment
import com.drtaa.core_map.moveCameraTo
import com.drtaa.core_map.setCustomLocationButton
import com.drtaa.core_model.map.Search
import com.drtaa.core_ui.hideKeyboard
import com.drtaa.core_ui.showSnackBar
import com.drtaa.feature_rent.adapter.SearchListAdapter
import com.drtaa.feature_rent.databinding.FragmentRentSearchBinding
import com.drtaa.feature_rent.viewmodel.RentSearchViewModel
import com.drtaa.feature_rent.viewmodel.RentViewModel
import com.google.android.material.bottomsheet.BottomSheetBehavior
import com.google.android.material.bottomsheet.BottomSheetBehavior.BottomSheetCallback
import com.naver.maps.geometry.LatLng
import com.naver.maps.map.MapView
import com.naver.maps.map.NaverMap
import com.naver.maps.map.util.FusedLocationSource
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.launch
import timber.log.Timber

@AndroidEntryPoint
class RentSearchFragment :
    BaseMapFragment<FragmentRentSearchBinding>(R.layout.fragment_rent_search) {
    private val rentViewModel: RentViewModel by hiltNavGraphViewModels(R.id.nav_graph_rent)
    private val rentSearchViewModel: RentSearchViewModel by viewModels()

    override var mapView: MapView? = null
    private var map: NaverMap? = null
    private lateinit var behavior: BottomSheetBehavior<ConstraintLayout>
    private lateinit var locationSource: FusedLocationSource

    private val searchListAdapter = SearchListAdapter()

    override fun initMapView() {
        mapView = binding.mvMap
        mapView?.getMapAsync(this)
    }

    override fun initOnMapReady(naverMap: NaverMap) {
        map = naverMap
        naverMap.uiSettings.isLocationButtonEnabled = false
        naverMap.moveCameraTo(jungryujang.latitude, jungryujang.longitude)
        naverMap.setCustomLocationButton(binding.layoutRentSearchBottomSheet.ivRentSearchCurrentLocation)
        locationSource = FusedLocationSource(this, LOCATION_PERMISSION_REQUEST_CODE)
        naverMap.addOnCameraIdleListener {
            val center = naverMap.cameraPosition.target
            viewLifecycleOwner.lifecycleScope.launch {
                rentSearchViewModel.getReverseGeocode(center.latitude, center.longitude)
                rentSearchViewModel.reverseGeocode.collect { result ->
                    result?.let {
                        it.onSuccess { title ->
                            binding.layoutRentSearchBottomSheet.etSearchLocation.setText(title)
                            rentSearchViewModel.setPinnedSearchItem(
                                Search(
                                    title, "",
                                    "",
                                    center.longitude,
                                    center.latitude
                                )
                            )
                        }.onFailure {
                            showSnackBar("주소를 가져오는데 실패 했습니다.")
                        }
                    }
                }
            }
        }
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
                rentSearchViewModel.setSelectedSearchItem(search)
                map?.moveCameraTo(search.lat, search.lng)
            }
        })

        binding.layoutRentSearchBottomSheet.rvSearchResult.adapter = searchListAdapter
    }

    private fun initObserve(naverMap: NaverMap) {
        rentSearchViewModel.selectedSearchItem.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { selectedSearchItem ->
                Timber.d("selectedSearchItem $selectedSearchItem")
                selectedSearchItem?.let {
                    naverMap.setMarker(selectedSearchItem.lat, selectedSearchItem.lng)
                    naverMap.setContentPadding(0, 0, 0, MAP_BOTTOM_CONTENT_PADDING)
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initObserve() {
        rentSearchViewModel.searchList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { result ->
                result.onSuccess { data ->
                    binding.layoutRentSearchBottomSheet.tvSearchBefore.visibility = View.GONE
                    searchListAdapter.submitList(data)
                    if (data.isEmpty()) {
                        binding.layoutRentSearchBottomSheet.tvSearchNothing.visibility =
                            View.VISIBLE
                        binding.layoutRentSearchBottomSheet.btnSearchSelect.visibility = View.GONE
                    } else {
                        binding.layoutRentSearchBottomSheet.tvSearchNothing.visibility = View.GONE
                        binding.layoutRentSearchBottomSheet.btnSearchSelect.visibility =
                            View.VISIBLE
                    }
                }.onFailure {
                    showSnackBar("오류가 발생했습니다. 다시 시도해주세요.")
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initEvent() {
        binding.layoutRentSearchBottomSheet.apply {
            this.ivSearchLocation.setOnClickListener {
                if (this.etSearchLocation.text.isEmpty()) {
                    showSnackBar("검색어를 입력해주세요.")
                    return@setOnClickListener
                }
                rentSearchViewModel.getSearchList(this.etSearchLocation.text.toString())
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
                if (rentSearchViewModel.selectedSearchItem.value != null) {
                    rentViewModel.setRentStartLocation(rentSearchViewModel.selectedSearchItem.value!!)
                    navigatePopBackStack()
                } else {
                    showSnackBar("장소를 선택해주세요.")
                }
            }
        }
    }

    private fun initBottomSheet() {
        binding.layoutRentSearchBottomSheet.tvSearchNothing.visibility = View.GONE
        binding.layoutRentSearchBottomSheet.btnSearchSelect.visibility = View.GONE

        behavior = BottomSheetBehavior.from(binding.clRentSearchBottomSheet)
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
//                Timber.d("드래그 중")
            }
        })
    }

    companion object {
        val jungryujang = LatLng(37.57578754990568, 126.90027478459672)
        const val BOTTOM_SHEET_PEEK_HEIGHT = 500
        const val MAP_BOTTOM_CONTENT_PADDING = 100
        const val LOCATION_PERMISSION_REQUEST_CODE = 1000
    }
}