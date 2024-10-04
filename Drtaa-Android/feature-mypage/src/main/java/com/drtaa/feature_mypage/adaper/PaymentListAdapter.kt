package com.drtaa.feature_mypage.adaper

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.drtaa.core_model.data.PaymentCompletionInfo
import com.drtaa.core_ui.base.BaseDiffUtil
import com.drtaa.feature_mypage.databinding.ItemPaymentListBinding

class PaymentListAdapter :
    ListAdapter<PaymentCompletionInfo, PaymentListAdapter.PaymentListViewHolder>(BaseDiffUtil<PaymentCompletionInfo>()){

    override fun onCreateViewHolder(
        parent: ViewGroup,
        viewType: Int
    ): PaymentListViewHolder {
        val binding = ItemPaymentListBinding.inflate(LayoutInflater.from(parent.context), parent, false)
        return PaymentListViewHolder(binding)
    }

    override fun onBindViewHolder(holder: PaymentListAdapter.PaymentListViewHolder, position: Int) {
        holder.bind(getItem(position))
    }

    inner class PaymentListViewHolder(private val binding: ItemPaymentListBinding) :
            RecyclerView.ViewHolder(binding.root) {
                fun bind(item: PaymentCompletionInfo) {
                    binding.completion = item
                    binding.executePendingBindings()
                }

    }

}