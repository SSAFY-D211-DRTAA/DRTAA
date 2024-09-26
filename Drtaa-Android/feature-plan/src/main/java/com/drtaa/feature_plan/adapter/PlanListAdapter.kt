package com.drtaa.feature_plan.adapter

import android.content.Context
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.appcompat.content.res.AppCompatResources
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.drtaa.core_model.plan.Plan.DayPlan.PlanItem
import com.drtaa.core_ui.base.BaseDiffUtil
import com.drtaa.feature_plan.databinding.ItemPlanBinding
import timber.log.Timber

class PlanListAdapter(
    private val context: Context,
    private val onPlanSelectListener: (planItem: PlanItem) -> Unit,
) :
    ListAdapter<PlanItem, PlanListAdapter.PlanItemViewHolder>(BaseDiffUtil<PlanItem>()),
    ItemTouchHelperListener {

    private var isEditMode: Boolean = false
    private val backgroundBlueCircle =
        AppCompatResources.getDrawable(context, com.drtaa.core_ui.R.drawable.circle_call)
    private val backgroundGrayCircle =
        AppCompatResources.getDrawable(context, com.drtaa.core_ui.R.drawable.circle_gray_d9d9)

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): PlanItemViewHolder {
        val binding = ItemPlanBinding.inflate(LayoutInflater.from(parent.context), parent, false)
        return PlanItemViewHolder(binding)
    }

    override fun onBindViewHolder(holder: PlanItemViewHolder, position: Int) {
        holder.bind(getItem(position), position + 1)
    }

    override fun onCurrentListChanged(
        previousList: MutableList<PlanItem>,
        currentList: MutableList<PlanItem>
    ) {
        super.onCurrentListChanged(previousList, currentList)
        Timber.d("previousList $previousList,\n currentList $currentList")
        if (previousList.size != currentList.size) {
            notifyDataSetChanged()
        }
    }

    inner class PlanItemViewHolder(private val binding: ItemPlanBinding) :
        RecyclerView.ViewHolder(binding.root) {

        fun bind(planItem: PlanItem, position: Int) {
            Timber.d("planItem $planItem")
            binding.plan = planItem
            binding.executePendingBindings()

            binding.tvPlanOrder.text = position.toString()
            setEditItemBackGround(planItem.isSelected)

            if (isEditMode) {
                binding.clPlanEditMode.visibility = View.VISIBLE
                binding.clPlanViewMode.visibility = View.GONE
            } else {
                binding.clPlanEditMode.visibility = View.GONE
                binding.clPlanViewMode.visibility = View.VISIBLE
            }

            binding.root.setOnClickListener {
                if (isEditMode) {
                    planItem.isSelected = !planItem.isSelected
                    onPlanSelectListener(planItem)
                    setEditItemBackGround(planItem.isSelected)
                }
            }
        }

        private fun setEditItemBackGround(isSelected: Boolean) {
            binding.viewCircleEdit.background =
                if (isSelected) {
                    backgroundBlueCircle
                } else {
                    backgroundGrayCircle
                }
        }
    }

    fun enableEditMode(isEditMode: Boolean) {
        Timber.d("isEditMode $isEditMode")
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
}