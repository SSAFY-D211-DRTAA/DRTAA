package com.drtaa.feature_tour

import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.paging.LoadState
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.core_ui.component.LocationHelper
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
    private val viewModel: TourViewModel by viewModels()
    private val tourAdapter by lazy {
        TourAdapter(onTourClickListener = {})
    }

    override fun initView() {
        initUI()
        getLocation()
        observeTourList()
    }

    private fun initUI() {
        binding.rvTour.adapter = tourAdapter
    }

    private fun getLocation() {
        viewLifecycleOwner.lifecycleScope.launch {
            locationHelper.getLastLocation().let { location ->
                location?.let {
                    viewModel.getLocationBasedList(
                        location.longitude.toString(),
//                        (128.4164712).toString(),
                        location.latitude.toString(),
                        "20000" // 반경 2키로 검색
                    )
                }
            }
        }
    }

    private fun observeTourList() {
        viewModel.pagedTour.flowWithLifecycle(viewLifecycleOwner.lifecycle)
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
}