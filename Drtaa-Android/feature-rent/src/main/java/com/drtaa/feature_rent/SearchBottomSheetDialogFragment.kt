package com.drtaa.feature_rent

import android.view.View
import androidx.constraintlayout.widget.ConstraintLayout
import com.drtaa.core_ui.base.BaseBottomSheetDialogFragment
import com.drtaa.feature_rent.databinding.FragmentSearchBottomSheetBinding
import com.google.android.material.bottomsheet.BottomSheetBehavior
import com.google.android.material.bottomsheet.BottomSheetBehavior.BottomSheetCallback
import timber.log.Timber

class SearchBottomSheetDialogFragment() :
    BaseBottomSheetDialogFragment<FragmentSearchBottomSheetBinding>(R.layout.fragment_search_bottom_sheet) {

    lateinit var behavior: BottomSheetBehavior<ConstraintLayout>

    override fun initView() {
        initBottomSheet()
    }

    private fun initBottomSheet() {
        behavior = BottomSheetBehavior.from(binding.clRootBottomSheet)
        behavior.isHideable = false

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
                Timber.d("드래그 중")
            }
        })
    }
}