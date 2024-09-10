package com.drtaa.core_model.util

import com.drtaa.core_model.data.Search
import com.drtaa.core_model.network.SearchItem

fun SearchItem.toSearch(): Search {
    return Search(
        title = this.title,
        category = this.category,
        roadAddress = this.roadAddress,
        mapx = this.mapx,
        mapy = this.mapy
    )
}