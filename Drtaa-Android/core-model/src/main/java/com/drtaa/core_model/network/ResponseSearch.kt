package com.drtaa.core_model.network

data class ResponseSearch(
    val display: Int,
    val items: List<SearchItem>,
    val lastBuildDate: String,
    val start: Int,
    val total: Int
)