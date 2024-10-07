package com.drtaa.core_model.map

import android.os.Parcel
import android.os.Parcelable

data class Search(
    val title: String,
    val category: String,
    val roadAddress: String,
    val lng: Double,
    val lat: Double
): Parcelable {
    constructor(parcel: Parcel) : this(
        parcel.readString() ?: "",
        parcel.readString() ?: "",
        parcel.readString() ?: "",
        parcel.readDouble(),
        parcel.readDouble()
    )

    override fun writeToParcel(parcel: Parcel, flags: Int) {
        parcel.writeString(title)
        parcel.writeString(category)
        parcel.writeString(roadAddress)
        parcel.writeDouble(lng)
        parcel.writeDouble(lat)
    }

    override fun describeContents(): Int {
        return 0
    }

    companion object CREATOR : Parcelable.Creator<Search> {
        override fun createFromParcel(parcel: Parcel): Search {
            return Search(parcel)
        }

        override fun newArray(size: Int): Array<Search?> {
            return arrayOfNulls(size)
        }
    }
}
