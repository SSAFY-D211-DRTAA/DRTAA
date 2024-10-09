from openai import OpenAI
import os

# OpenAI API 키 설정
OPENAI_API_KEY = "key"

client = OpenAI(
#   api_key=os.environ["$OPENAI_API_KEY"],  # this is also the default, it can be omitted
    api_key=OPENAI_API_KEY
)

def get_recommendations(region):
    # 프롬프트에 변수 'region'을 삽입
    prompt = f"{region} 근처에서 방문할 만한 장소를 추천해줘."

    completion = client.chat.completions.create(model="gpt-4o-mini",
        messages=[{"role": "system", "content": "You are a helpful assistant."},
                  {"role": "user", "content": prompt}])


    # 생성된 텍스트 추출
    print(completion)
    generated_text = completion.choices[0].message.content
    print(completion.usage)
    print(generated_text)
    return generated_text

# 파일에서 텍스트 읽어오기 함수
def read_region_from_file(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            region = file.read().strip()  # 파일에서 지역명 텍스트를 읽어옴
            return region
    except FileNotFoundError:
        print("파일을 찾을 수 없습니다.")
        return None

# 파일 경로 지정
file_path = "output/stt.txt"

# 파일에서 지역명 가져오기
region_name = read_region_from_file(file_path)

# 지역명이 정상적으로 읽혔을 때만 LLM 호출
if region_name:
    recommendations = get_recommendations(region_name)
    print(f"{region_name} 근처 추천 장소: {recommendations}")