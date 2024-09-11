package com.drtaa.feature_rent

import android.view.View
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_map.base.BaseMapFragment
import com.drtaa.core_ui.showSnackBar
import com.drtaa.feature_rent.adapter.SearchListAdapter
import com.drtaa.feature_rent.databinding.FragmentRentSearchBinding
import com.drtaa.feature_rent.viewmodel.RentViewModel
import com.google.android.material.bottomsheet.BottomSheetBehavior
import com.google.android.material.bottomsheet.BottomSheetBehavior.BottomSheetCallback
import com.naver.maps.map.MapView
import com.naver.maps.map.NaverMap
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import timber.log.Timber

@AndroidEntryPoint
class RentSearchFragment :
    BaseMapFragment<FragmentRentSearchBinding>(R.layout.fragment_rent_search) {
    private val viewModel: RentViewModel by viewModels()
    override var mapView: MapView? = null

    private lateinit var behavior: BottomSheetBehavior<ConstraintLayout>

    private val searchListAdapter = SearchListAdapter()

    override fun initMapView() {
        mapView = binding.mvMap
        mapView?.getMapAsync(this)
    }

    override fun initOnMapReady(naverMap: NaverMap) {
        // 지도 작업
    }

    override fun iniView() {
        initBottomSheet()
        initEvent()
        initObserve()
        initRV()
    }

    private fun initRV() {

        binding.layoutRentSearchBottomSheet.rvSearchResult.adapter = searchListAdapter
    }

    private fun initObserve() {
        viewModel.searchList.flowWithLifecycle(viewLifecycleOwner.lifecycle).onEach { result ->
            result.onSuccess { data ->
                binding.layoutRentSearchBottomSheet.tvSearchBefore.visibility = View.GONE
                searchListAdapter.submitList(data)
                if (data.isEmpty()) {
                    binding.layoutRentSearchBottomSheet.tvSearchNothing.visibility = View.VISIBLE
                } else {
                    binding.layoutRentSearchBottomSheet.tvSearchNothing.visibility = View.GONE
                }
            }.onFailure {
                // 실패 메세지 추가
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
                viewModel.getSearchList(this.etSearchLocation.text.toString())
            }
        }
    }

    private fun initBottomSheet() {
        binding.layoutRentSearchBottomSheet.tvSearchNothing.visibility = View.GONE

        behavior = BottomSheetBehavior.from(binding.clRentSearchBottomSheet)
        behavior.state = BottomSheetBehavior.STATE_EXPANDED
        behavior.isHideable = false // 완전히 숨길 수 없도록 설정

        behavior.peekHeight = BOTTOM_SHEET_PEEK_HEIGHT // 바텀시트가 접혔을 때의 높이 (적절하게 설정)

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
        const val BOTTOM_SHEET_PEEK_HEIGHT = 500
    }
}