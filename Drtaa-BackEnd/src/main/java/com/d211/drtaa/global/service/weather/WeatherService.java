package com.d211.drtaa.global.service.weather;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

import java.util.Map;

@Service
public class WeatherService {

    @Value("${weather.api-key}")
    private String apiKey;

    public Map<String, Object> getWeekWeather(double latitude, double longitude) throws Exception {
        String apiUrl = "https://api.openweathermap.org/data/2.5/forecast";
        String requestUrl = String.format("%s?lat=%f&lon=%f&appid=%s&units=metric&cnt=40", apiUrl, latitude, longitude, apiKey);

        RestTemplate restTemplate = new RestTemplate();
        Map<String, Object> response = restTemplate.getForObject(requestUrl, Map.class);
        return response;
    }
}