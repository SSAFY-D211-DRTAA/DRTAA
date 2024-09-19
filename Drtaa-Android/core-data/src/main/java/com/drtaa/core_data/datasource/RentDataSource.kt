package com.drtaa.core_data.datasource

import com.drtaa.core_model.network.RequestDispatch
import com.drtaa.core_model.network.RequestDrivingCar
import com.drtaa.core_model.network.ResponseRentCar

interface RentDataSource {
    suspend fun getUnassignedCar(): ResponseRentCar
    suspend fun getAssignedCar(): ResponseRentCar
    suspend fun getAssignedCar(rentCarId: Int): ResponseRentCar
    suspend fun getDrivingCar(rentCarId: Int): ResponseRentCar
    suspend fun patchDispatchCar(request: RequestDispatch): ResponseRentCar
    suspend fun getAllDispatchCar(): ResponseRentCar
    suspend fun patchDriveCar(request: RequestDrivingCar): ResponseRentCar
}