package com.drtaa.core_ui.component

import android.content.Context
import com.drtaa.core_ui.R
import com.drtaa.core_ui.base.BaseDialog
import com.drtaa.core_ui.databinding.DialogTypingTwoButtonBinding

class TwoButtonTypingDialog(
    val context: Context,
    private val defaultText: String,
    private val onCheckClick: ((newText: String) -> Unit)? = null
) : BaseDialog<DialogTypingTwoButtonBinding>(context, R.layout.dialog_typing_two_button) {
    override fun onCreateDialog() {
        initMessage()
        initEvent()
    }

    private fun initMessage() {
        binding.etDialogMsg.setText(defaultText)
    }

    private fun initEvent() {
        binding.btnCheck.setOnClickListener {
            onCheckClick?.let { it(binding.etDialogMsg.text.toString()) }
            dismiss()
        }

        binding.btnCancel.setOnClickListener {
            dismiss()
        }
    }
}