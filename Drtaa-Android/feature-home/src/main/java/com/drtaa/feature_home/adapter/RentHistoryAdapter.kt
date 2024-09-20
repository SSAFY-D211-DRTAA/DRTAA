package com.drtaa.feature_home.adapter

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.drtaa.core_model.rent.RentSimple
import com.drtaa.core_ui.base.BaseDiffUtil
import com.drtaa.feature_home.databinding.ItemRentHistoryBinding

class RentHistoryAdapter :
    ListAdapter<RentSimple, RentHistoryAdapter.RentHistoryViewHolder>(BaseDiffUtil<RentSimple>()) {

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): RentHistoryViewHolder {
        val binding =
            ItemRentHistoryBinding.inflate(LayoutInflater.from(parent.context), parent, false)
        return RentHistoryViewHolder(binding)
    }

    override fun onBindViewHolder(holder: RentHistoryViewHolder, position: Int) {
        holder.bind(getItem(position))
    }

    inner class RentHistoryViewHolder(private val binding: ItemRentHistoryBinding) :
        RecyclerView.ViewHolder(binding.root) {
        fun bind(rentSimple: RentSimple) {
            binding.rent = rentSimple
            binding.executePendingBindings()

            binding.root.setOnClickListener {
                itemClickListener.onItemClicked(rentSimple)
            }
        }
    }

    interface ItemClickListener {
        fun onItemClicked(rentSimple: RentSimple)
    }

    private lateinit var itemClickListener: ItemClickListener

    fun setItemClickListener(itemClickListener: ItemClickListener) {
        this.itemClickListener = itemClickListener
    }
}