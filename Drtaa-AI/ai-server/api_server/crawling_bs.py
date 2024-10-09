import requests
from bs4 import BeautifulSoup
import json

url = "https://api.visitkorea.or.kr/#/useInforService"

# 웹페이지 요청
response = requests.get(url)
soup = BeautifulSoup(response.text, 'html.parser')

# 소분류 정보가 있는 테이블 찾기
table = soup.find('table', {'class': 'table table-bordered'})

# 소분류 정보 추출
categories = []
if table:
    rows = table.find_all('tr')[1:]  # 헤더 행 제외
    for row in rows:
        cols = row.find_all('td')
        if len(cols) >= 2:
            code = cols[0].text.strip()
            name = cols[1].text.strip()
            categories.append({"code": code, "name": name})

# JSON 파일로 저장
with open('subcategories.json', 'w', encoding='utf-8') as f:
    json.dump(categories, f, ensure_ascii=False, indent=4)

print("소분류 정보가 subcategories.json 파일로 저장되었습니다.")
