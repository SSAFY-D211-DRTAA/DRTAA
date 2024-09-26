package com.drtaa.feature_plan.adapter

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.drtaa.core_model.map.Search
import com.drtaa.core_ui.base.BaseDiffUtil
import com.drtaa.feature_plan.databinding.ItemSearchPlanBinding

class SearchListAdapter :
    ListAdapter<Search, SearchListAdapter.SearchViewHolder>(BaseDiffUtil<Search>()) {

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): SearchViewHolder {
        val binding = ItemSearchPlanBinding.inflate(LayoutInflater.from(parent.context), parent, false)
        return SearchViewHolder(binding)
    }

    override fun onBindViewHolder(holder: SearchViewHolder, position: Int) {
        val isLastItem = position == itemCount - 1
        holder.bind(getItem(position), isLastItem)
    }

    inner class SearchViewHolder(private val binding: ItemSearchPlanBinding) :
        RecyclerView.ViewHolder(binding.root) {
        fun bind(search: Search, isLastItem: Boolean) {
            binding.search = search

            if (isLastItem) {
                binding.viewSearchItemDivider.visibility = View.GONE
            } else {
                binding.viewSearchItemDivider.visibility = View.VISIBLE
            }
            binding.executePendingBindings()

            binding.root.setOnClickListener {
                itemClickListener.onItemClicked(search)
            }
        }
    }

    interface ItemClickListener {
        fun onItemClicked(search: Search)
    }

    private lateinit var itemClickListener: ItemClickListener

    fun setItemClickListener(itemClickListener: ItemClickListener) {
        this.itemClickListener = itemClickListener
    }
}