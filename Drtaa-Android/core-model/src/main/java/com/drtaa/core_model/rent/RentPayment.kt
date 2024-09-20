package com.drtaa.core_model.rent

import android.os.Parcel
import android.os.Parcelable

data class RentPayment(
    val orderName: String,
    val orderId: String,
    val items: List<Product>
) : Parcelable {
    constructor(parcel: Parcel) : this(
        parcel.readString() ?: "",
        parcel.readString() ?: "",
        parcel.createTypedArrayList(Product.CREATOR) ?: listOf()
    )

    override fun writeToParcel(parcel: Parcel, flags: Int) {
        parcel.writeString(orderName)
        parcel.writeTypedList(items)
    }

    override fun describeContents(): Int {
        return 0
    }

    companion object CREATOR : Parcelable.Creator<RentPayment> {
        override fun createFromParcel(parcel: Parcel): RentPayment {
            return RentPayment(parcel)
        }

        override fun newArray(size: Int): Array<RentPayment?> {
            return arrayOfNulls(size)
        }
    }

    data class Product(
        val name: String,
        val id: String,
        val price: Int,
        val quantity: Int
    ) : Parcelable {
        constructor(parcel: Parcel) : this(
            parcel.readString() ?: "",
            parcel.readString() ?: "",
            parcel.readInt(),
            parcel.readInt()
        )

        override fun writeToParcel(parcel: Parcel, flags: Int) {
            parcel.writeString(name)
            parcel.writeString(id)
            parcel.writeInt(price)
            parcel.writeInt(quantity)
        }

        override fun describeContents(): Int {
            return 0
        }

        companion object CREATOR : Parcelable.Creator<Product> {
            override fun createFromParcel(parcel: Parcel): Product {
                return Product(parcel)
            }

            override fun newArray(size: Int): Array<Product?> {
                return arrayOfNulls(size)
            }
        }
    }
}