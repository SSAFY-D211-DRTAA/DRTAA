package com.drtaa.feature_rent

import android.content.Context
import android.widget.NumberPicker
import com.drtaa.core_ui.base.BaseDialog
import com.drtaa.feature_rent.databinding.DialogTimePickerBinding
import okhttp3.internal.format

class TimePickerDialog(
    val context: Context
) : BaseDialog<DialogTimePickerBinding>(context, R.layout.dialog_time_picker) {

    private var hour: Int = DEFAULT_HOUR
    private var minute: Int = DEFAULT_MINUTE

    private val hourArray = Array(HOUR_LENGTH) { i -> format("%02d", i) }
    private val minuteArray = arrayOf("00", "30")

    override fun onCreateDialog() {
        initTimePicker()
        initEvent()
    }

    private fun initTimePicker() {
        binding.npTimePickerHour.apply {
            displayedValues = hourArray
            minValue = 0
            maxValue = hourArray.size - 1
            descendantFocusability = NumberPicker.FOCUS_BLOCK_DESCENDANTS
        }
        binding.npTimePickerMinute.apply {
            displayedValues = minuteArray
            minValue = 0
            maxValue = minuteArray.size - 1
            descendantFocusability = NumberPicker.FOCUS_BLOCK_DESCENDANTS
        }

        setView()
    }

    private fun initEvent() {
        binding.npTimePickerHour.setOnValueChangedListener { _, _, newVal ->
            hour = Integer.parseInt(hourArray[newVal])
        }

        binding.npTimePickerMinute.setOnValueChangedListener { _, _, newVal ->
            minute = Integer.parseInt(minuteArray[newVal])
        }

        binding.btnCheck.setOnClickListener {
            onCheckClickListener.onCheckClick(
                hour,
                minute
            )
            dismiss()
        }

        binding.btnCancel.setOnClickListener {
            dismiss()
        }
    }

    fun setTime(hour: Int, minute: Int) {
        this.hour = hour
        this.minute = minute

        setView()
    }

    private fun setView() {
        binding.npTimePickerHour.value =
            hourArray.indexOf(hour.toString().padStart(2, '0'))

        val minuteStr = if (minute == 0) "00" else "30"
        binding.npTimePickerMinute.value = minuteArray.indexOf(minuteStr)
    }

    interface OnCheckClickListener {
        fun onCheckClick(hour: Int, minute: Int)
    }

    lateinit var onCheckClickListener: OnCheckClickListener

    companion object {
        const val HOUR_LENGTH = 24
        const val DEFAULT_HOUR = 9
        const val DEFAULT_MINUTE = 0
    }
}