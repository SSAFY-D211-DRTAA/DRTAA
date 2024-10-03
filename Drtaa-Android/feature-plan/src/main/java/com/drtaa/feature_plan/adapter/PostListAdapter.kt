package com.drtaa.feature_plan.adapter

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.drtaa.core_model.travel.NaverPost
import com.drtaa.core_ui.base.BaseDiffUtil
import com.drtaa.feature_plan.databinding.ItemTravelBlogBinding

class PostListAdapter(
    private val onClick: (NaverPost) -> Unit
) :
    ListAdapter<NaverPost, PostListAdapter.PostViewHolder>(BaseDiffUtil<NaverPost>()) {

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): PostViewHolder {
        val binding =
            ItemTravelBlogBinding.inflate(LayoutInflater.from(parent.context), parent, false)
        return PostViewHolder(binding)
    }

    override fun onBindViewHolder(holder: PostViewHolder, position: Int) {
        holder.bind(getItem(position))
    }

    inner class PostViewHolder(private val binding: ItemTravelBlogBinding) :
        RecyclerView.ViewHolder(binding.root) {
        fun bind(post: NaverPost) {
            binding.post = post

            binding.root.setOnClickListener {
                onClick(post)
            }
        }
    }
}
