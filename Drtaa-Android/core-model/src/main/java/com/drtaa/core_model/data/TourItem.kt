package com.drtaa.core_model.data

import kotlinx.serialization.SerialName

data class TourItem(
    @SerialName("contentid")
    val contentid: String,
    @SerialName("addr2")
    val addr2: String,
    @SerialName("firstimage2")
    val firstimage2: String,
    @SerialName("cpyrhtDivCd")
    val cpyrhtDivCd: String,
    @SerialName("addr1")
    val addr1: String,
    @SerialName("contenttypeid")
    val contenttypeid: String,
    @SerialName("createdtime")
    val createdtime: String,
    @SerialName("dist")
    val dist: String,
    @SerialName("firstimage")
    val firstimage: String,
    @SerialName("areacode")
    val areacode: String,
    @SerialName("booktour")
    val booktour: String,
    @SerialName("mapx")
    val mapx: String,
    @SerialName("mapy")
    val mapy: String,
    @SerialName("mlevel")
    val mlevel: String,
    @SerialName("modifiedtime")
    val modifiedtime: String,
    @SerialName("sigungucode")
    val sigungucode: String,
    @SerialName("tel")
    val tel: String,
    @SerialName("title")
    val title: String,
    @SerialName("cat1")
    val cat1: String,
    @SerialName("cat2")
    val cat2: String,
    @SerialName("cat3")
    val cat3: String
)
