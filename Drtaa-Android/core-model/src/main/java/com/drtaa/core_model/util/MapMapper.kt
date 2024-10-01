package com.drtaa.core_model.util

import com.drtaa.core_model.map.Search
import com.drtaa.core_model.network.SearchItem
import com.drtaa.core_model.plan.Plan.DayPlan.PlanItem

fun SearchItem.toSearch(): Search {
    return Search(
        title = this.title.removeHtmlTags().removeHtmlEntities(),
        category = this.category.removeTextBeforeArrow(),
        roadAddress = this.roadAddress,
        lng = this.mapx.toDouble() / 10000000,
        lat = this.mapy.toDouble() / 10000000
    )
}

fun Search.toPlanItem(travelDatesId: Int): PlanItem {
    return PlanItem(
        datePlacesAddress = this.roadAddress,
        datePlacesCategory = this.category,
        datePlacesIsVisited = false,
        datePlacesLat = this.lat,
        datePlacesLon = this.lng,
        datePlacesName = this.title,
        datePlacesOrder = 0,
        datePlacesId = 0,
        travelDatesId = travelDatesId,
    )
}

/**
 * HTML 태그를 찾아 제거
 */
fun String.removeHtmlTags(): String {
    return this.replace(Regex("<.*?>"), "")
}

/**
 * HTML 엔티티를 찾아 제거
 */
fun String.removeHtmlEntities(): String {
    return this
        .replace("&amp;", "&")
        .replace("&lt;", "<")
        .replace("&gt;", ">")
        .replace("&quot;", "\"")
        .replace("&apos;", "'")
        .replace("&#39;", "'")
        .replace("&#34;", "\"")
        .replace("&nbsp;", " ")
}

/**
 * 첫 글자부터 ">"까지의 문자열을 제거
 */
fun String.removeTextBeforeArrow(): String {
    return this.substringAfterLast(">")
}