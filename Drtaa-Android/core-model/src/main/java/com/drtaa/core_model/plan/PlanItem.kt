package com.drtaa.core_model.plan

import android.os.Parcel
import android.os.Parcelable

data class PlanItem(
    val travelDatesId: Int,
    val datePlacesAddress: String,
    val datePlacesCategory: String,
    val datePlacesIsVisited: Boolean,
    val datePlacesLat: Double,
    val datePlacesLon: Double,
    val datePlacesName: String,
    val datePlacesId: Int = 0,
    val datePlacesOrder: Int,
    var isSelected: Boolean = false
) : Parcelable {
    constructor(parcel: Parcel) : this(
        parcel.readInt(),
        parcel.readString() ?: "",
        parcel.readString() ?: "",
        parcel.readByte() != 0.toByte(),
        parcel.readDouble(),
        parcel.readDouble(),
        parcel.readString() ?: "",
        parcel.readInt(),
        parcel.readInt(),
        parcel.readByte() != 0.toByte()
    )

    override fun writeToParcel(parcel: Parcel, flags: Int) {
        parcel.writeInt(travelDatesId)
        parcel.writeString(datePlacesAddress)
        parcel.writeString(datePlacesCategory)
        parcel.writeByte(if (datePlacesIsVisited) 1 else 0)
        parcel.writeDouble(datePlacesLat)
        parcel.writeDouble(datePlacesLon)
        parcel.writeString(datePlacesName)
        parcel.writeInt(datePlacesId)
        parcel.writeInt(datePlacesOrder)
        parcel.writeByte(if (isSelected) 1 else 0)
    }

    override fun describeContents(): Int {
        return 0
    }

    companion object CREATOR : Parcelable.Creator<PlanItem> {
        override fun createFromParcel(parcel: Parcel): PlanItem {
            return PlanItem(parcel)
        }

        override fun newArray(size: Int): Array<PlanItem?> {
            return arrayOfNulls(size)
        }
    }
}
