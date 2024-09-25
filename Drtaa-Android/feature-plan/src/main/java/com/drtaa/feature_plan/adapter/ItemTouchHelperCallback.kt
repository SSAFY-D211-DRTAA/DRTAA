package com.drtaa.feature_plan.adapter

import androidx.recyclerview.widget.ItemTouchHelper
import androidx.recyclerview.widget.RecyclerView

class ItemTouchHelperCallback(
    val listener: ItemTouchHelperListener
) : ItemTouchHelper.Callback() {

    // 활성화된 이동 방향을 정의하는 플래그를 반환하는 메소드
    override fun getMovementFlags(
        recyclerView: RecyclerView,
        viewHolder: RecyclerView.ViewHolder
    ): Int {
        // 드래그 방향
        val dragFlags = ItemTouchHelper.UP or ItemTouchHelper.DOWN

        // 이동을 만드는 메소드
        return makeMovementFlags(dragFlags, 0)
    }

    // 드래그된 item을 이전 위치에서 새로운 위치로 옮길 때 호출
    override fun onMove(
        recyclerView: RecyclerView,
        viewHolder: RecyclerView.ViewHolder,
        target: RecyclerView.ViewHolder
    ): Boolean {
//        if (viewHolder.itemView.y < 100) {
//            recyclerView.smoothScrollBy(0, -20)  // 위쪽 스크롤
//        } else if (viewHolder.itemView.y > recyclerView.height - 100) {
//            recyclerView.smoothScrollBy(0, 20)  // 아래쪽 스크롤
//        }

        return listener.onItemMove(viewHolder.adapterPosition, target.adapterPosition)
    }

    override fun onSwiped(viewHolder: RecyclerView.ViewHolder, direction: Int) {
        // 스와이프 될 때
    }
}