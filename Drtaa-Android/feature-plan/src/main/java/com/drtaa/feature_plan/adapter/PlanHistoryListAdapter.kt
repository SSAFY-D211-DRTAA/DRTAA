package com.drtaa.feature_plan.adapter

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.drtaa.core_model.plan.PlanSimple
import com.drtaa.core_ui.base.BaseDiffUtil
import com.drtaa.feature_plan.databinding.ItemPlanHistoryBinding

class PlanHistoryListAdapter :
    ListAdapter<PlanSimple, PlanHistoryListAdapter.PlanHistoryViewHolder>(BaseDiffUtil<PlanSimple>()) {

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): PlanHistoryViewHolder {
        val binding =
            ItemPlanHistoryBinding.inflate(LayoutInflater.from(parent.context), parent, false)
        return PlanHistoryViewHolder(binding)
    }

    override fun onBindViewHolder(holder: PlanHistoryViewHolder, position: Int) {
        holder.bind(getItem(position))
    }

    inner class PlanHistoryViewHolder(private val binding: ItemPlanHistoryBinding) :
        RecyclerView.ViewHolder(binding.root) {
        fun bind(planSimple: PlanSimple) {
            binding.planSimple = planSimple
            binding.executePendingBindings()

            binding.root.setOnClickListener {
                itemClickListener.onItemClicked(
                    travelId = planSimple.travelId,
                    rentId = planSimple.rentId
                )
            }
        }
    }

    interface ItemClickListener {
        fun onItemClicked(travelId: Int, rentId: Int)
    }

    private lateinit var itemClickListener: ItemClickListener

    fun setItemClickListener(itemClickListener: ItemClickListener) {
        this.itemClickListener = itemClickListener
    }
}