package com.drtaa.core_model.map

data class ResponseCarRoute(
    val msg: Path,
) {
    data class Path(
        val path: List<CarRoute>,
    )
}
