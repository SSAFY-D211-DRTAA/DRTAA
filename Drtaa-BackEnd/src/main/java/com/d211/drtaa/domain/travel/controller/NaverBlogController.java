package com.d211.drtaa.domain.travel.controller;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.jsoup.Jsoup;
import org.jsoup.safety.Safelist;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.io.*;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLEncoder;
import java.util.*;

@RestController
@RequestMapping("/travel")
@RequiredArgsConstructor
@CrossOrigin("*")
@Tag(name = "블로그 컨트롤러", description = "네이버 블로그 크롤링")
@Log4j2
public class NaverBlogController {

    @Value("${naver.client-id}")
    private String clientId; // 애플리케이션 클라이언트 아이디
    @Value("${naver.client-secret}")
    private String clientSecret; // 애플리케이션 클라이언트 시크릿

    @GetMapping(value = "/blog/{keyword}", produces = "application/json; charset=utf-8")
    @CrossOrigin(origins = "*", methods = RequestMethod.GET)
    @Operation(summary = "")
    public ResponseEntity<String> getUserNews(@PathVariable("keyword") String keyword) {
        String text = null;
        try {
            text = URLEncoder.encode(keyword, "UTF-8");
        } catch (UnsupportedEncodingException e) {
            throw new RuntimeException("검색어 인코딩 실패", e);
        }

        String apiURL = "https://openapi.naver.com/v1/search/blog?query=" + text + "&display=3"; // JSON 결과

        Map<String, String> requestHeaders = new HashMap<>();
        requestHeaders.put("X-Naver-Client-Id", clientId);
        requestHeaders.put("X-Naver-Client-Secret", clientSecret);
        String responseBody = get(apiURL, requestHeaders);
        log.info("Naver Blog Get Success !!");

        List<Map<String, Object>> blogList = new ArrayList<>();

        try {
            // 응답 문자열을 JSON 형식으로 변환
            ObjectMapper objectMapper = new ObjectMapper();
            Map<String, Object> jsonMap = objectMapper.readValue(responseBody, new TypeReference<Map<String, Object>>(){});

            // 블로그 항목이 있을 경우
            if (jsonMap.containsKey("items") && jsonMap.get("items") instanceof List) {
                List<Map<String, Object>> items = (List<Map<String, Object>>) jsonMap.get("items");

                for (Map<String, Object> item : items) {
                    if (item.containsKey("title")) {
                        item.put("title", cleanHtml((String) item.get("title")));
                    }
                    if (item.containsKey("description")) {
                        item.put("description", cleanHtml((String) item.get("description")));
                    }
                    if (item.containsKey("originallink")) {
                        item.put("originallink", cleanHtml((String) item.get("originallink")));
                    }
                    if (item.containsKey("link")) {
                        item.put("link", cleanHtml((String) item.get("link")));
                    }
                }
                blogList.addAll(items);
            }

            // 전체 응답을 JSON 배열로 변환
            responseBody = objectMapper.writeValueAsString(blogList);

        } catch (Exception e) {
            throw new RuntimeException("JSON 처리 실패", e);
        }

        return new ResponseEntity<>(responseBody, HttpStatus.OK);
    }

    private static String get(String apiUrl, Map<String, String> requestHeaders) {
        HttpURLConnection con = connect(apiUrl);
        try {
            con.setRequestMethod("GET");
            for (Map.Entry<String, String> header : requestHeaders.entrySet()) {
                con.setRequestProperty(header.getKey(), header.getValue());
            }

            int responseCode = con.getResponseCode();
            if (responseCode == HttpURLConnection.HTTP_OK) { // 정상 호출
                return readBody(con.getInputStream());
            } else { // 오류 발생
                return readBody(con.getErrorStream());
            }
        } catch (IOException e) {
            throw new RuntimeException("API 요청과 응답 실패", e);
        } finally {
            con.disconnect();
        }
    }

    private static HttpURLConnection connect(String apiUrl) {
        try {
            URL url = new URL(apiUrl);
            return (HttpURLConnection) url.openConnection();
        } catch (MalformedURLException e) {
            throw new RuntimeException("API URL이 잘못되었습니다. : " + apiUrl, e);
        } catch (IOException e) {
            throw new RuntimeException("연결이 실패했습니다. : " + apiUrl, e);
        }
    }

    private static String readBody(InputStream body) {
        InputStreamReader streamReader = new InputStreamReader(body);

        try (BufferedReader lineReader = new BufferedReader(streamReader)) {
            StringBuilder responseBody = new StringBuilder();

            String line;
            while ((line = lineReader.readLine()) != null) {
                responseBody.append(line);
            }

            return responseBody.toString();
        } catch (IOException e) {
            throw new RuntimeException("API 응답을 읽는 데 실패했습니다.", e);
        }
    }

    private String cleanHtml(String html) {
        // HTML 태그 제거
        String cleanedHtml = Jsoup.clean(html, Safelist.none());
        // HTML 엔티티 디코딩
        cleanedHtml = Jsoup.parse(cleanedHtml).text();
        // 남아있을 수 있는 &quot; 등의 엔티티 추가 처리
        cleanedHtml = cleanedHtml.replace("&quot;", "\"")
                .replace("&amp;", "&")
                .replace("&lt;", "<")
                .replace("&gt;", ">")
                .replace("&nbsp;", " ");
        return cleanedHtml;
    }
}
