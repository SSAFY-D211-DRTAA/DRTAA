package com.drtaa.core_ui.component

import android.content.Context
import com.drtaa.core_ui.R
import com.drtaa.core_ui.base.BaseDialog
import com.drtaa.core_ui.databinding.DialogMsgTwoButtonBinding

class TwoButtonMessageDialog(
    val context: Context,
    private val message: String,
    private val onCheckClick: (() -> Unit)? = null
) : BaseDialog<DialogMsgTwoButtonBinding>(context, R.layout.dialog_msg_two_button) {
    override fun onCreateDialog() {
        initMessage()
        initEvent()
    }

    private fun initMessage() {
        binding.tvDialogMsg.text = message
    }

    private fun initEvent() {
        binding.btnCheck.setOnClickListener {
            onCheckClick?.let { it() }
            dismiss()
        }

        binding.btnCancel.setOnClickListener {
            dismiss()
        }
    }
}