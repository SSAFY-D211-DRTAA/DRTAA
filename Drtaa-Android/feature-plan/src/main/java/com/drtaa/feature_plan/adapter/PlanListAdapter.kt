package com.drtaa.feature_plan.adapter

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.lifecycle.MutableLiveData
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.drtaa.core_model.plan.Plan.DayPlan.PlanItem
import com.drtaa.core_ui.base.BaseDiffUtil
import com.drtaa.feature_plan.databinding.ItemPlanBinding
import kotlinx.coroutines.flow.MutableStateFlow
import timber.log.Timber

class PlanListAdapter :
    ListAdapter<PlanItem, PlanListAdapter.PlanItemViewHolder>(BaseDiffUtil<PlanItem>()),
    ItemTouchHelperListener {

    private var isEditMode: Boolean = false

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): PlanItemViewHolder {
        val binding = ItemPlanBinding.inflate(LayoutInflater.from(parent.context), parent, false)
        return PlanItemViewHolder(binding)
    }

    override fun onBindViewHolder(holder: PlanItemViewHolder, position: Int) {
        holder.bind(getItem(position), position)
    }

    inner class PlanItemViewHolder(private val binding: ItemPlanBinding) :
        RecyclerView.ViewHolder(binding.root) {
        fun bind(planItem: PlanItem, position: Int) {
            binding.plan = planItem
            binding.executePendingBindings()

            binding.tvPlanOrder.text = (position + 1).toString()

            if (isEditMode) {
                binding.clPlanEditMode.visibility = View.VISIBLE
                binding.clPlanViewMode.visibility = View.GONE
            } else {
                binding.clPlanEditMode.visibility = View.GONE
                binding.clPlanViewMode.visibility = View.VISIBLE
            }

//            binding.root.setOnClickListener {
//                itemClickListener.onItemClicked(planItem)
//            }
        }
    }

    interface ItemClickListener {
        fun onItemClicked(planItem: PlanItem)
    }

    private lateinit var itemClickListener: ItemClickListener

    fun setItemClickListener(itemClickListener: ItemClickListener) {
        this.itemClickListener = itemClickListener
    }

    fun enableEditMode(isEditMode: Boolean) {
        this.isEditMode = isEditMode
        notifyDataSetChanged()
    }

    override fun onItemMove(from: Int, to: Int): Boolean {
        // currentList를 가변 리스트로 변환
        val mutableList = currentList.toMutableList()
        val planItem = mutableList[from]

        Timber.d("from $from, to $to, planItem $planItem")

        // 아이템 이동
        mutableList.removeAt(from)
        mutableList.add(to, planItem)

        // 새로운 리스트로 submit
        submitList(mutableList)
        return true
    }

    override fun onItemSwipe(position: Int) {
        // currentList를 가변 리스트로 변환
        val mutableList = currentList.toMutableList()

        mutableList.removeAt(position)
        // 새로운 리스트로 submit
        submitList(mutableList)
    }
}