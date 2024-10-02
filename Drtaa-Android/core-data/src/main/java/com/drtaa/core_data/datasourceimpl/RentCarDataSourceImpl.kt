package com.drtaa.core_data.datasourceimpl

import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.intPreferencesKey
import com.drtaa.core_data.datasource.RentCarDataSource
import com.drtaa.core_model.network.RequestCarStatus
import com.drtaa.core_model.network.RequestDrivingCar
import com.drtaa.core_model.network.RequestRentCarCall
import com.drtaa.core_model.network.RequestUnassignedCar
import com.drtaa.core_model.network.ResponseDrivingCar
import com.drtaa.core_model.network.ResponseRentCarCall
import com.drtaa.core_model.rent.RentCar
import com.drtaa.core_model.rent.RentTravelInfo
import com.drtaa.core_network.api.RentCarAPI
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.map
import javax.inject.Inject
import javax.inject.Named

class RentCarDataSourceImpl @Inject constructor(
    @Named("RENT")
    private val dataStore: DataStore<Preferences>,
    private val rentCarAPI: RentCarAPI,
) : RentCarDataSource {
    override suspend fun getUnassignedCar(rentSchedule: RequestUnassignedCar): RentCar {
        return rentCarAPI.getUnassignedCar(rentSchedule)
    }

    override suspend fun getOffCar(rentInfo: RequestCarStatus): String {
        return rentCarAPI.getOffCar(rentInfo)
    }

    override suspend fun getOnCar(rentInfo: RequestCarStatus): RentTravelInfo {
        return rentCarAPI.getOnCar(rentInfo)
    }

    override suspend fun callAssignedCar(requestCallCar: RequestRentCarCall): ResponseRentCarCall {
        return rentCarAPI.callAssignedCar(requestCallCar)
    }

    override suspend fun callFirstAssignedCar(rentId: Long): ResponseRentCarCall {
        return rentCarAPI.callFirstAssignedCar(rentId)
    }

    override suspend fun editDriveStatus(request: RequestDrivingCar): ResponseDrivingCar {
        return rentCarAPI.editDriveStatus(request)
    }

    override suspend fun getDriveStatus(rentCarId: Long): ResponseDrivingCar {
        return rentCarAPI.getDriveStatus(rentCarId)
    }

    override suspend fun getCarWithTravelInfo(): RequestCarStatus {
        return dataStore.data.map { prefs ->
            RequestCarStatus(
                prefs[RENT_ID] ?: -1,
                prefs[TRAVEL_ID] ?: -1,
                prefs[TRAVEL_DATES_ID] ?: -1,
                prefs[DATE_PLACES_ID] ?: -1,
            )
        }.first()
    }

    override suspend fun setCarWithTravelInfo(rentInfo: RequestCarStatus) {
        dataStore.edit { prefs ->
            prefs[RENT_ID] = rentInfo.rentId
            prefs[TRAVEL_ID] = rentInfo.travelId
            prefs[TRAVEL_DATES_ID] = rentInfo.travelDatesId
            prefs[DATE_PLACES_ID] = rentInfo.datePlacesId
        }
    }

    companion object {
        val RENT_ID = intPreferencesKey("rentId")
        val TRAVEL_ID = intPreferencesKey("travelId")
        val TRAVEL_DATES_ID = intPreferencesKey("travelDatesId")
        val DATE_PLACES_ID = intPreferencesKey("datePlacesId")
    }
}