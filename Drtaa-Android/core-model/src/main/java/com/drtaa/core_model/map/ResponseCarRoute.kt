package com.drtaa.core_model.map

data class ResponseCarRoute(
    val tag: String,
    val msg: Path,
) {
    data class Path(
        val path: List<CarRoute>,
    )
}
