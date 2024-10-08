package com.drtaa.feature_mypage

import android.view.View
import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_model.rent.RentSimple
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.core_ui.expandLayout
import com.drtaa.core_ui.foldLayout
import com.drtaa.feature_mypage.adaper.RentHistoryAdapter
import com.drtaa.feature_mypage.databinding.FragmentRentHistoryBinding
import com.drtaa.feature_mypage.viewmodel.RentHistoryViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach

@AndroidEntryPoint
class RentHistoryFragment :
    BaseFragment<FragmentRentHistoryBinding>(R.layout.fragment_rent_history) {

    private val rentHistoryViewModel: RentHistoryViewModel by viewModels()

    private val rentReservedListAdapter = RentHistoryAdapter()
    private val rentCompletedListAdapter = RentHistoryAdapter()

    override fun onResume() {
        super.onResume()
        rentHistoryViewModel.getRentList()
    }

    override fun initView() {
        initRVAdapter()
        initObserve()
        initEvent()
    }

    private fun initEvent() {
        binding.apply {
            llRentReserved.setOnClickListener {
                if (clRentReserved.visibility == View.GONE) {
                    expandLayout(ivReservedExpand, clRentReserved)
                } else {
                    foldLayout(ivReservedExpand, clRentReserved)
                }
            }

            llRentCompleted.setOnClickListener {
                if (clRentCompleted.visibility == View.GONE) {
                    expandLayout(ivCompletedExpand, clRentCompleted)
                } else {
                    foldLayout(ivCompletedExpand, clRentCompleted)
                }
            }

            cvRentInProgress.setOnClickListener {
                rentHistoryViewModel.rentInProgress.value?.let {
                    moveToSummaryFragment(it.rentId.toLong())
                }
            }
        }
    }

    private fun initObserve() {
        rentHistoryViewModel.rentList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { rentList ->
                if (rentList == null) {
                    binding.tvErrorRentHistory.visibility = View.VISIBLE
                    binding.clRent.visibility = View.GONE
                } else {
                    binding.tvErrorRentHistory.visibility = View.GONE
                    binding.clRent.visibility = View.VISIBLE
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        rentHistoryViewModel.rentInProgress.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { rentInProgress ->
                if (rentInProgress == null) {
                    binding.cvRentInProgress.visibility = View.GONE
                    binding.clRentNoInProgress.visibility = View.VISIBLE
                } else {
                    binding.cvRentInProgress.visibility = View.VISIBLE
                    binding.clRentNoInProgress.visibility = View.GONE

                    binding.rent = rentInProgress
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        rentHistoryViewModel.rentReservedList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { reservedList ->
                if (reservedList == null) return@onEach

                if (reservedList.isEmpty()) {
                    binding.clRentNoReserved.visibility = View.VISIBLE
                    binding.rvRentReservedHistory.visibility = View.GONE
                } else {
                    binding.clRentNoReserved.visibility = View.GONE
                    binding.rvRentReservedHistory.visibility = View.VISIBLE

                    rentReservedListAdapter.submitList(reservedList)
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        rentHistoryViewModel.rentCompletedList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { completedList ->
                if (completedList == null) return@onEach

                if (completedList.isEmpty()) {
                    binding.clRentNoCompleted.visibility = View.VISIBLE
                    binding.rvRentCompletedHistory.visibility = View.GONE
                } else {
                    binding.clRentNoCompleted.visibility = View.GONE
                    binding.rvRentCompletedHistory.visibility = View.VISIBLE

                    rentCompletedListAdapter.submitList(completedList)
                }
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initRVAdapter() {
        val clickListener = object : RentHistoryAdapter.ItemClickListener {
            override fun onItemClicked(rentSimple: RentSimple) {
                moveToSummaryFragment(rentSimple.rentId.toLong())
            }
        }
        rentReservedListAdapter.setItemClickListener(clickListener)
        rentCompletedListAdapter.setItemClickListener(clickListener)

        binding.rvRentReservedHistory.adapter = rentReservedListAdapter
        binding.rvRentCompletedHistory.adapter = rentCompletedListAdapter

        binding.rvRentReservedHistory.itemAnimator = null
        binding.rvRentCompletedHistory.itemAnimator = null
    }

    private fun moveToSummaryFragment(rentId: Long) {
        val action =
            RentHistoryFragmentDirections.actionRentHistoryFragmentToRentHistorySummaryFragment(
                rentId = rentId
            )
        navigateDestination(action)
    }
}