package com.drtaa.feature_tour.component

import android.view.View
import androidx.recyclerview.widget.RecyclerView.ViewHolder
import com.drtaa.core_model.tour.TourItem
import com.drtaa.core_ui.centerCrop
import com.drtaa.feature_tour.databinding.ItemTourBinding

class TourViewHolder(
    private val binding: ItemTourBinding,
    private val onTourClickListener: (TourItem) -> Unit,
) : ViewHolder(binding.root) {

    fun bind(data: TourItem) {
        binding.apply {
            tvTourTitle.text = data.title
            tvTourTel.text = data.tel
            tvTourAddr.text = data.addr1
            if (data.firstimage.isNotEmpty()) {
                ivTour.visibility = View.VISIBLE
                ivTour.centerCrop(data.firstimage, binding.root.context)
            } else {
                ivTour.visibility = View.GONE
            }
            itemView.setOnClickListener {
                onTourClickListener(data)
            }
        }
    }
}