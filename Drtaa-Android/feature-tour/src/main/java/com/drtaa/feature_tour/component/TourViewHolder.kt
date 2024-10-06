package com.drtaa.feature_tour.component

import android.view.View
import androidx.recyclerview.widget.RecyclerView.ViewHolder
import com.drtaa.core_model.tour.TourItem
import com.drtaa.core_ui.centerCrop
import com.drtaa.feature_tour.R
import com.drtaa.feature_tour.databinding.ItemTourBinding

class TourViewHolder(
    private val binding: ItemTourBinding,
    private val onTourClickListener: (TourItem) -> Unit,
) : ViewHolder(binding.root) {

    fun bind(data: TourItem) {
        binding.apply {
            tvTourTitle.text = data.title
            if (data.tel.isEmpty() || data.tel.length == 0) {
                imgTourTel.visibility = View.GONE
                tvTourTel.visibility = View.GONE
            } else {
                imgTourTel.visibility = View.VISIBLE
                tvTourTel.visibility = View.VISIBLE
                tvTourTel.text = data.tel
            }

            tvTourAddr.text = data.addr1
            if (data.firstimage.isNotBlank()) {
                ivTour.centerCrop(data.firstimage, binding.root.context)
            } else {
                ivTour.setImageResource(com.drtaa.core_ui.R.drawable.ic_tour_basic_image)
            }
            itemView.setOnClickListener {
                onTourClickListener(data)
            }
        }
    }
}