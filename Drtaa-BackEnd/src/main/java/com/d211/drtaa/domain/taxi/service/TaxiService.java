package com.d211.drtaa.domain.taxi.service;

import com.d211.drtaa.domain.taxi.dto.request.TaxiCreateRequestDTO;
import com.d211.drtaa.domain.taxi.dto.response.TaxiDetailResponseDTO;

public interface TaxiService {
    TaxiDetailResponseDTO createTaxi(String userProviderId, TaxiCreateRequestDTO rentCreateRequestDTO);
}
