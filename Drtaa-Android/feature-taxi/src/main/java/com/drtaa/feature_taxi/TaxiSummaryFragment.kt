package com.drtaa.feature_taxi

import android.graphics.Color
import androidx.fragment.app.viewModels
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.navigation.fragment.findNavController
import com.drtaa.core_map.base.BaseMapFragment
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.rent.Payment
import com.drtaa.core_model.util.Pay
import com.drtaa.core_ui.showSnackBar
import com.drtaa.feature_taxi.databinding.FragmentTaxiSummaryBinding
import com.drtaa.feature_taxi.viewmodel.TaxiSummaryViewModel
import com.drtaa.feature_taxi.viewmodel.TaxiViewModel
import com.naver.maps.geometry.LatLng
import com.naver.maps.geometry.LatLngBounds
import com.naver.maps.map.CameraUpdate
import com.naver.maps.map.MapView
import com.naver.maps.map.NaverMap
import com.naver.maps.map.overlay.ArrowheadPathOverlay
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.launch
import timber.log.Timber
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter

@AndroidEntryPoint
class TaxiSummaryFragment :
    BaseMapFragment<FragmentTaxiSummaryBinding>(R.layout.fragment_taxi_summary) {

    private val taxiViewModel: TaxiViewModel by hiltNavGraphViewModels(R.id.nav_graph_taxi)
    private val taxiSummaryViewModel: TaxiSummaryViewModel by viewModels()

    override var mapView: MapView? = null

    private var pathOverlay: ArrowheadPathOverlay? = null

    private lateinit var naverMap: NaverMap

    override fun initMapView() {
        mapView = binding.mvTaxiSummary
        mapView?.getMapAsync(this)
    }

    override fun initOnMapReady(naverMap: NaverMap) {
        this.naverMap = naverMap
        naverMap.uiSettings.isLocationButtonEnabled = false

        initObserve()
        observeDialog()
        requestRoute()
    }

    override fun iniView() {
        initEvent()
        initData()
    }

    private fun initData() {
        val taxiStartLocation = taxiViewModel.taxiStartLocation.value
        val taxiEndLocation = taxiViewModel.taxiEndLocation.value
        if (taxiStartLocation != null && taxiEndLocation != null) {
            taxiSummaryViewModel.apply {
                setTaxiStartLocation(taxiStartLocation)
                setTaxiEndLocation(taxiEndLocation)
            }
        }

        val taxiSchedule = RequestUnassignedCar(
            rentCarScheduleStartDate = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd")),
            rentCarScheduleEndDate = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd"))
        )
        taxiSummaryViewModel.getUnAssignedCar(taxiSchedule)
        Timber.d("택시 정보는요 ${taxiViewModel.taxiInfo}")
    }

    private fun initObserve() {
        taxiViewModel.routeOverlay.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { routePoints ->
                routePoints?.let {
                    drawRoute(it)
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        taxiViewModel.routeInfo.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { routeInfo ->
                routeInfo?.let {
                    binding.apply {
                        tvTaxiDuration.text = "${routeInfo.totalTime / Hour}분"
                        tvTaxiPrice.text = "${routeInfo.taxiFare}원"
                    }
                    taxiViewModel.getTaxiInfo()
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun drawRoute(routePoints: List<LatLng>) {
        pathOverlay?.map = null
        pathOverlay = ArrowheadPathOverlay().apply {
            coords = routePoints
            map = naverMap
            width = WIDTH
            color = Color.BLUE
            outlineWidth = OUTLINE_WIDTH
            outlineColor = Color.WHITE
        }

        val bounds = LatLngBounds.Builder().include(routePoints).build()
        naverMap.moveCamera(CameraUpdate.fitBounds(bounds, PADDING))
    }

    private fun requestRoute() {
        val start = taxiViewModel.taxiStartLocation.value
        val end = taxiViewModel.taxiEndLocation.value
        if (start != null && end != null) {
            taxiViewModel.getRoute(start, end)
        }
    }

    private fun initEvent() {
        binding.btnTaxiSummaryBack.setOnClickListener {
            navigatePopBackStack()
        }

        binding.btnTaxiSummaryPayment.setOnClickListener {
            navigationToPayment()
        }
    }

    private fun navigationToPayment() {
        val action = TaxiSummaryFragmentDirections.actionTaxiSummaryFragmentToPaymentFragment(
            Payment(
                "DRTAA 택시 이용",
                "2",
                listOf(
                    Payment.Product(
                        "DRTAA 택시",
                        "TAXI_CODE",
                        PADDING,
                        1
                    )
                )
            )
        )
        clearBackStackEntryState()
        navigateDestination(action)
    }

    private fun observeDialog() {
        viewLifecycleOwner.lifecycleScope.launch {
            launch {
                findNavController()
                    .currentBackStackEntry
                    ?.savedStateHandle
                    ?.getStateFlow(Pay.SUCCESS.type, Pair(false, ""))
                    ?.collectLatest { (success, paymentData) ->
                        if (success) {
                            showSnackBar("결제에 성공했습니다.")
                            Timber.tag("bootpay").d("택시용 ${Pay.SUCCESS.type} : $paymentData")
                            taxiSummaryViewModel.processBootpayPayment(
                                paymentData,
                                taxiViewModel.taxiInfo.value!!
                            )
                        }
                    }
            }
            launch {
                findNavController()
                    .currentBackStackEntry
                    ?.savedStateHandle
                    ?.getStateFlow(Pay.CLOSED.type, false)
                    ?.collectLatest { closed ->
                        if (closed) {
                            showSnackBar("결제가 취소되었습니다.")
                            Timber.tag("bootpay").d("택시용 ${Pay.CLOSED.type} : $closed")
                        }
                    }
            }
            launch {
                findNavController()
                    .currentBackStackEntry
                    ?.savedStateHandle
                    ?.getStateFlow(Pay.CANCELED.type, false)
                    ?.collectLatest { canceled ->
                        if (canceled) {
                            showSnackBar("결제가 취소되었습니다.")
                            Timber.tag("bootpay").d("택시용 ${Pay.CANCELED.type} : $canceled")
                        }
                    }
            }
        }
    }

    private fun clearBackStackEntryState() {
        val savedStateHandle = findNavController().currentBackStackEntry?.savedStateHandle ?: return

        val states = mapOf(
            Pay.SUCCESS.type to Pair(false, ""),
            Pay.CLOSED.type to false,
            Pay.CANCELED.type to false
        )

        states.forEach { (key, value) ->
            savedStateHandle[key] = value
        }
    }

    companion object {
        const val Hour = 60
        const val PADDING = 100
        const val WIDTH = 20
        const val OUTLINE_WIDTH = 2
    }
}