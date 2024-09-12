package com.drtaa.core_model.network


import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

@Serializable
data class ResponseTour(
    @SerialName("response")
    val response: Response
) {
    @Serializable
    data class Response(
        @SerialName("header")
        val header: Header,
        @SerialName("body")
        val body: Body
    ) {
        @Serializable
        data class Header(
            @SerialName("resultCode")
            val resultCode: String,
            @SerialName("resultMsg")
            val resultMsg: String
        )

        @Serializable
        data class Body(
            @SerialName("items")
            val items: Items,
            @SerialName("numOfRows")
            val numOfRows: Int,
            @SerialName("pageNo")
            val pageNo: Int,
            @SerialName("totalCount")
            val totalCount: Int
        ) {
            @Serializable
            data class Items(
                @SerialName("item")
                val item: List<Item>
            ) {
                @Serializable
                data class Item(
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
            }
        }
    }
}