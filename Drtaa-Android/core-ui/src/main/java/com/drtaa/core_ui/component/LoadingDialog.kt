package com.drtaa.core_ui.component

import android.content.Context
import com.drtaa.core_ui.R
import com.drtaa.core_ui.base.BaseDialog
import com.drtaa.core_ui.databinding.DialogLoadingBinding

class LoadingDialog(context: Context): BaseDialog<DialogLoadingBinding>(context, R.layout.dialog_loading) {
    override fun onCreateDialog() {
        this.setCancelable(false)
    }
}