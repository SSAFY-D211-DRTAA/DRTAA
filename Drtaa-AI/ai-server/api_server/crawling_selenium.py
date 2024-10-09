from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from selenium.webdriver.chrome.service import Service
from webdriver_manager.chrome import ChromeDriverManager
import json
import time

# Selenium 설정
options = webdriver.ChromeOptions()
# options.add_argument('--headless')  # 헤드리스 모드 (화면 표시 없음)
driver = webdriver.Chrome(service=Service(ChromeDriverManager().install()), options=options)

# 웹페이지 접속
url = "https://api.visitkorea.or.kr/#/useInforService"
driver.get(url)

# 페이지 로드 대기
WebDriverWait(driver, 10).until(
    EC.presence_of_element_located((By.CLASS_NAME, "btn-type1"))
)

# 검색 버튼 클릭
search_button = driver.find_element(By.CLASS_NAME, "btn-type1")
search_button.click()

# 결과 로딩 대기
time.sleep(5)  # 동적 로딩을 위한 대기 시간

# 테이블 데이터 추출
table_selector = "#wrap > section > div.content.inner.page-apiguide.page-openapiguide > div.tab-content.tab-content6.on > article > div.table-wrap.scroll-x2 > table > tbody > tr"
rows = WebDriverWait(driver, 10).until(
    EC.presence_of_all_elements_located((By.CSS_SELECTOR, table_selector))
)

categories = []
for row in rows:
    cols = row.find_elements(By.TAG_NAME, "td")
    code = cols[5].text.strip()  # 6번째 열
    name = cols[6].text.strip()  # 7번째 열
    categories.append({"code": code, "name": name})

# JSON 파일로 저장
with open('categories.json', 'w', encoding='utf-8') as f:
    json.dump(categories, f, ensure_ascii=False, indent=4)

print("서비스 분류코드가 categories.json 파일로 저장되었습니다.")

# 브라우저 종료
driver.quit()
