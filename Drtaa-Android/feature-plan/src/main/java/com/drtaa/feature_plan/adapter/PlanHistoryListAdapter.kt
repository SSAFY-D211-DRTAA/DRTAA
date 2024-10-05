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

            binding.ivPlanStatus.setImageResource(
                initImageStatus(planSimple)
            )

            binding.root.setOnClickListener {
                itemClickListener.onItemClicked(
                    travelId = planSimple.travelId,
                    rentId = planSimple.rentId
                )
            }
        }

        private fun initImageStatus(planSimple: PlanSimple) =
            when (planSimple.rentStatus) {
                "in_progress" -> {
                    com.drtaa.core_ui.R.drawable.ic_in_progress
                }

                "reserved" -> {
                    com.drtaa.core_ui.R.drawable.ic_reserved
                }

                else -> {
                    com.drtaa.core_ui.R.drawable.ic_completed
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