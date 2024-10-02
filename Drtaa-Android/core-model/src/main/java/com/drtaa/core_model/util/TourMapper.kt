package com.drtaa.core_model.util

import com.drtaa.core_model.tour.TourItem
import com.drtaa.core_model.network.ResponseTour
import com.drtaa.core_model.plan.PlanItem

fun (ResponseTour.Response.Body.Items.Item).toEntity(): TourItem {
    return TourItem(
        contentid = this.contentid,
        addr2 = this.addr2,
        firstimage2 = this.firstimage2,
        cpyrhtDivCd = this.cpyrhtDivCd,
        addr1 = this.addr1,
        contenttypeid = this.contenttypeid,
        createdtime = this.createdtime,
        dist = this.dist,
        firstimage = this.firstimage,
        areacode = this.areacode,
        booktour = this.booktour,
        mapx = this.mapx,
        mapy = this.mapy,
        mlevel = this.mlevel,
        modifiedtime = this.modifiedtime,
        sigungucode = this.sigungucode,
        tel = this.tel.removeHtmlTags(),
        title = this.title,
        cat1 = this.cat1,
        cat2 = this.cat2,
        cat3 = this.cat3
    )
}

fun TourItem.toPlanItem(): PlanItem {
    return PlanItem(
        travelDatesId = 0,
        datePlacesAddress = this.addr1,
        datePlacesCategory = this.cat1,
        datePlacesIsVisited = false,
        datePlacesLat = this.mapy.toDouble(),
        datePlacesLon = this.mapx.toDouble(),
        datePlacesName = this.title,
        datePlacesId = 0,
        datePlacesOrder = 0
    )
}