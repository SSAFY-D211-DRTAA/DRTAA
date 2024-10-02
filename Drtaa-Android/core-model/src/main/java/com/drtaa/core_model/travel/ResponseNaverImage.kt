package com.drtaa.core_model.travel

data class ResponseNaverImage(
    val display: Int,
    val items: List<NaverImage>,
    val lastBuildDate: String,
    val start: Int,
    val total: Int
)