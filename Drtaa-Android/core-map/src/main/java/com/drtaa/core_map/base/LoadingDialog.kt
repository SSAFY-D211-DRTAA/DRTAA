package com.drtaa.core_map.base

import android.content.Context
import com.drtaa.core_map.R
import com.drtaa.core_map.databinding.DialogLoadingBinding

class LoadingDialog(context: Context) :
    BaseDialog<DialogLoadingBinding>(context, R.layout.dialog_loading) {
    override fun onCreateDialog() {
        this.setCancelable(false)
    }
}