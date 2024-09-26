package com.drtaa.feature_plan

import android.content.Context
import android.widget.NumberPicker
import com.drtaa.core_ui.base.BaseDialog
import com.drtaa.feature_plan.databinding.DialogDatePickerBinding

class DatePickerDialog(
    val context: Context,
    private val dateList: Array<String>,
    private var dayIdx: Int
) : BaseDialog<DialogDatePickerBinding>(context, R.layout.dialog_date_picker) {

    private var date: String = dateList[dayIdx]

    override fun onCreateDialog() {
        initDatePicker()
        initEvent()
    }

    private fun initDatePicker() {
        binding.npDatePicker.apply {
            displayedValues = dateList
            minValue = 0
            maxValue = dateList.size - 1
            descendantFocusability = NumberPicker.FOCUS_BLOCK_DESCENDANTS
        }
        setView()
    }

    private fun initEvent() {
        binding.npDatePicker.setOnValueChangedListener { _, _, newVal ->
            date = dateList[newVal]
            dayIdx = newVal
        }

        binding.btnCheck.setOnClickListener {
            onCheckClickListener.onCheckClick(dayIdx)
            dismiss()
        }

        binding.btnCancel.setOnClickListener {
            dismiss()
        }
    }

    fun setDate(dayIdx: Int) {
        this.dayIdx = dayIdx

        setView()
    }

    private fun setView() {
        binding.npDatePicker.value = dayIdx
    }

    interface OnCheckClickListener {
        fun onCheckClick(selectedDateIdx: Int)
    }

    lateinit var onCheckClickListener: OnCheckClickListener
}