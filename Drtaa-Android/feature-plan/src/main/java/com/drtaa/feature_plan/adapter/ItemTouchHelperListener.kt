package com.drtaa.feature_plan.adapter

interface ItemTouchHelperListener {
    fun onItemMove(from: Int, to: Int): Boolean
}