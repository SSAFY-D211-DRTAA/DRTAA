package com.drtaa.feature_tour.component

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.paging.PagingDataAdapter
import androidx.recyclerview.widget.RecyclerView.ViewHolder
import com.drtaa.core_model.data.TourItem
import com.drtaa.core_ui.base.BaseDiffUtil
import com.drtaa.feature_tour.databinding.ItemTourBinding

class TourAdapter(
    private val onTourClickListener: (TourItem) -> Unit,
) : PagingDataAdapter<TourItem, ViewHolder>(BaseDiffUtil<TourItem>()) {
    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
        val layoutInflater = LayoutInflater.from(parent.context)
        val binding = ItemTourBinding.inflate(layoutInflater, parent, false)
        return TourViewHolder(
            binding,
            onTourClickListener = onTourClickListener,
        )
    }

    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        when (holder) {
            is TourViewHolder -> {
                val item = getItem(position)
                item?.let {
                    holder.bind(item)
                }
            }
        }
    }
}