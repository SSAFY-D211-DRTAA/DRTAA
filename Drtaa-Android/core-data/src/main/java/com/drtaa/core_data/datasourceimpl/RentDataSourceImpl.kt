package com.drtaa.core_data.datasourceimpl

import com.drtaa.core_data.datasource.RentDataSource
import com.drtaa.core_model.network.RequestDispatch
import com.drtaa.core_model.network.RequestDrivingCar
import com.drtaa.core_model.network.ResponseRentCar
import com.drtaa.core_network.api.RentAPI
import javax.inject.Inject

class RentDataSourceImpl @Inject constructor(
    private val rentAPI: RentAPI
) : RentDataSource {
    override suspend fun getUnassignedCar(): ResponseRentCar {
        return rentAPI.getUnassignedCar()
    }

    override suspend fun getAssignedCar(): ResponseRentCar {
        TODO("Not yet implemented")
    }

    override suspend fun getAssignedCar(rentCarId: Int): ResponseRentCar {
        TODO("Not yet implemented")
    }

    override suspend fun getDrivingCar(rentCarId: Int): ResponseRentCar {
        TODO("Not yet implemented")
    }

    override suspend fun patchDispatchCar(request: RequestDispatch): ResponseRentCar {
        TODO("Not yet implemented")
    }

    override suspend fun getAllDispatchCar(): ResponseRentCar {
        TODO("Not yet implemented")
    }

    override suspend fun patchDriveCar(request: RequestDrivingCar): ResponseRentCar {
        TODO("Not yet implemented")
    }
}