package com.drtaa.feature_rent.adapter

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.drtaa.core_model.data.Search
import com.drtaa.core_ui.base.BaseDiffUtil
import com.drtaa.feature_rent.databinding.ItemSearchBinding

class SearchListAdapter :
    ListAdapter<Search, SearchListAdapter.SearchViewHolder>(BaseDiffUtil<Search>()) {

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): SearchViewHolder {
        val binding = ItemSearchBinding.inflate(LayoutInflater.from(parent.context), parent, false)
        return SearchViewHolder(binding)
    }

    override fun onBindViewHolder(holder: SearchViewHolder, position: Int) {
        holder.bind(getItem(position))
    }

    class SearchViewHolder(private val binding: ItemSearchBinding) :
        RecyclerView.ViewHolder(binding.root) {
        fun bind(search: Search) {
            binding.search = search
//            binding.executePendingBindings()
        }

    }

}