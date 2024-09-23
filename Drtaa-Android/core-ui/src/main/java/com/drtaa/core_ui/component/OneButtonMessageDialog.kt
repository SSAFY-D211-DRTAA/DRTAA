package com.drtaa.core_ui.component

import android.content.Context
import com.drtaa.core_ui.R
import com.drtaa.core_ui.base.BaseDialog
import com.drtaa.core_ui.databinding.DialogMsgOneButtonBinding

class OneButtonMessageDialog(
    val context: Context,
    private var message: String,
    private val onCheckClick: (() -> Unit)? = null
) : BaseDialog<DialogMsgOneButtonBinding>(context, R.layout.dialog_msg_one_button) {
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
    }
}