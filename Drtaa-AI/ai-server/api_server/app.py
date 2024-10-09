import boto3
import json
import logging
import time
import os

from dotenv import load_dotenv
from flask import Flask, redirect, request
from flask_cors import CORS
from flask_restx import Api, Resource, fields
from langchain.prompts import PromptTemplate
from langchain.schema import HumanMessage
from langchain.prompts import ChatPromptTemplate
from langchain.output_parsers import CommaSeparatedListOutputParser
from langchain_openai import ChatOpenAI

from botocore.exceptions import ClientError

from firebase_place_manager import write_place

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

load_dotenv()

OPENAI_API_KEY = os.getenv('OPENAI_API_KEY')
AWS_ACCESS_KEY = os.getenv('AWS_ACCESS_KEY')
AWS_SECRET_KEY = os.getenv('AWS_SECRET_KEY')
AWS_REGION = os.getenv('AWS_REGION', 'your-region')
DYNAMODB_TABLE = os.getenv('DYNAMODB_TABLE', 'your-table-name')

# 환경 변수 검증
# if not all([OPENAI_API_KEY, AWS_ACCESS_KEY, AWS_SECRET_KEY, AWS_REGION, DYNAMODB_TABLE]):
if not all([OPENAI_API_KEY]):
    raise ValueError("필요한 환경 변수가 설정되지 않았습니다.")

app = Flask(__name__)
CORS(app)

api = Api(app, version='1.0', 
          title='AI API 문서', 
          description='Swagger 문서', 
          doc="/ai-api-server/api-docs", 
          prefix='/ai-api-server')

llm = ChatOpenAI(api_key=OPENAI_API_KEY, 
                 model="gpt-4o-mini")

try:
    dynamodb = boto3.resource('dynamodb', 
                              region_name=AWS_REGION,
                              aws_access_key_id=AWS_ACCESS_KEY,
                              aws_secret_access_key=AWS_SECRET_KEY)
    table = dynamodb.Table(DYNAMODB_TABLE)
except ClientError as e:
    logger.error(f"DynamoDB 연결 실패: {e}")
    raise

prompt_template = PromptTemplate(
    input_variables=["query"],
    template="사용자 질문: {query}\n\n답변:"
)

# 장소 추천을 위한 프롬프트 템플릿 정의
recommend_template = ChatPromptTemplate.from_template(
    "다음은 여행지 목록입니다:\n\n{places}\n\n"
    "사용자의 요구사항: {user_request}\n\n"
    "위의 여행지 목록에서 사용자의 요구사항에 가장 적합한 장소를 1개 추천해주세요. "
    "각 추천에 대한 간단한 이유도 함께 제시해주세요. "
    "응답 형식은 다음과 같아야 합니다:\n"
    "장소명: title\n추천 이유: reason"
)

def save_conversation(user_id, query, response):
    try:
        table.put_item(
            Item={
                'user_id': user_id,
                'timestamp': int(time.time()),
                'query': query,
                'response': response
            }
        )
    except ClientError as e:
        logger.error(f"DynamoDB에 저장 실패: {e}")
        raise

# 모델 정의
request_model = api.model('Request', {
    'query': fields.String(required=True, description='사용자 질문'),
    'user_id': fields.String(required=False, description='사용자 ID')
})

response_model = api.model('Response', {
    'response': fields.String(description='AI의 응답')
})

# 새로운 모델 정의
place_model = api.model('Place', {
    'address': fields.String(required=True, description='주소'),
    'category': fields.String(required=True, description='카테고리'),
    'datePlacesLat': fields.String(required=True, description='위도'),
    'datePlacesLon': fields.String(required=True, description='경도'),
    'datePlacesName': fields.String(required=True, description='장소 이름')
})

# 장소 추천 모델 정의
recommend_model = api.model('RecommendRequest', {
    'user_request': fields.String(required=True, description='사용자의 여행 요구사항')
})

# JSON 파일 로드 함수
def load_place_data():
    # current_dir = os.path.dirname(os.path.abspath(__file__))
    # json_path = os.path.join(current_dir, 'place_data.json')
    with open('place_data.json', 'r', encoding='utf-8') as file:
        return json.load(file)

# 전역 변수로 place_data 로드
place_data = load_place_data()

def load_categories():
    with open('categories.json', 'r', encoding='utf-8') as file:
        return json.load(file)

# 전역 변수로 categories 로드
categories = load_categories()

def code_to_name(code):
    for category in categories:
        if category['code'] == code:
            return category['name']
    return code  # 일치하는 이름이 없으면 원래 코드 반환

@api.route('/')
class Home(Resource):
    def get(self):
        return redirect('/ai-api-server/api-docs')

@api.route('/process_request')
class ProcessRequest(Resource):
    @api.expect(request_model)
    @api.marshal_with(response_model, code=200)
    @api.response(400, 'Invalid request format')
    @api.response(500, 'Internal server error')
    def post(self):
        """Process user query and return AI response"""
        try:
            data = api.payload
            user_query = data['query']
            user_id = data.get('user_id', 'anonymous')

            prompt = prompt_template.format(query=user_query)
            messages = [HumanMessage(content=prompt)]
            
            response = llm.invoke(messages)
            response_content = response.content

            # save_conversation(user_id, user_query, response_content)
            return {"response": response_content}
        
        except KeyError as e:
            logger.error(f"Missing key in request: {e}")
            api.abort(400, f"Invalid request format: {str(e)}")
        except Exception as e:
            logger.error(f"Error processing request: {e}")
            api.abort(500, f"Internal server error: {str(e)}")

@api.route('/recommend_place')
class RecommendPlaces(Resource):
    @api.expect(recommend_model)
    @api.response(200, 'Success')
    @api.response(400, 'Invalid request format')
    @api.response(500, 'Internal server error')
    def post(self):
        """Recommend places based on user request"""
        try:
            data = api.payload
            user_request = data['user_request']

            # place_data에서 필요한 정보만 추출
            places_info = [f"{place['title']} - {place['addr1']} - {code_to_name(place['cat3'])}" for place in place_data['items']]
            places_str = "\n".join(places_info)

            # 프롬프트 생성
            prompt = recommend_template.format(places=places_str, user_request=user_request)
            messages = [HumanMessage(content=prompt)]

            # GPT-4를 사용하여 추천 생성
            response = llm.invoke(messages)
            recommendation = response.content
            logger.info(recommendation)
            
            
            # 추천된 장소 정보 추출
            recommended_place_name = recommendation.split("\n")[0].split(":")[1].strip()
            
            # place_data에서 추천된 장소 찾기
            recommended_place = next((place for place in place_data['items'] if place['title'] == recommended_place_name), None)

            logger.info(recommended_place)

            if recommended_place:
                # WritePlace 요청 수행
                write_place_data = {
                    'address': recommended_place['addr1'],
                    'category': code_to_name(recommended_place.get('cat3', '')),  # 코드를 이름으로 변환
                    'datePlacesLat': recommended_place['mapy'],
                    'datePlacesLon': recommended_place['mapx'],
                    'datePlacesName': recommended_place['title']
                }
                
                # WritePlace 요청 수행
                write_place_response = self.write_place(write_place_data)

                return {
                    "recommendation": recommendation,
                    "write_place_response": write_place_response
                }, 200
            else:
                return {"error": "Recommended place not found in the database"}, 404

        except KeyError as e:
            logger.error(f"Missing key in request: {e}")
            api.abort(400, f"Invalid request format: {str(e)}")
        except Exception as e:
            logger.error(f"Error processing recommendation request: {e}")
            api.abort(500, f"Internal server error: {str(e)}")

    def write_place(self, data):
        """Internal method to call WritePlace"""
        try:
            write_place(
                data['address'],
                data['category'],
                data['datePlacesLat'],
                data['datePlacesLon'],
                data['datePlacesName']
            )
            return "Place information successfully written"
        except Exception as e:
            logger.error(f"Error writing place information: {e}")
            return f"Error writing place information: {str(e)}"

@api.route('/places')
class Places(Resource):
    @api.response(200, 'Success')
    @api.response(500, 'Internal server error')
    def get(self):
        """Get all places from loaded JSON data"""
        try:
            return {"places": place_data['items']}, 200
        except Exception as e:
            logger.error(f"Error retrieving place data: {e}")
            api.abort(500, f"Internal server error: {str(e)}")

@api.route('/place/<int:index>')
class PlaceByIndex(Resource):
    @api.response(200, 'Success')
    @api.response(404, 'Place not found')
    @api.response(500, 'Internal server error')
    def get(self, index):
        """Get a specific place by index"""
        try:
            if 0 <= index < len(place_data['items']):
                return place_data['items'][index], 200
            else:
                api.abort(404, "Place not found")
        except Exception as e:
            logger.error(f"Error retrieving place data: {e}")
            api.abort(500, f"Internal server error: {str(e)}")

@api.route('/place/content/<string:contentid>')
class Place(Resource):
    @api.response(200, 'Success')
    @api.response(404, 'Place not found')
    @api.response(500, 'Internal server error')
    def get(self, contentid):
        """Get a specific place by contentid"""
        try:
            place = next((item for item in place_data['items'] if item['contentid'] == contentid), None)
            if place:
                return place, 200
            else:
                api.abort(404, "Place not found")
        except Exception as e:
            logger.error(f"Error retrieving place data: {e}")
            api.abort(500, f"Internal server error: {str(e)}")

@api.route('/write_place')
class WritePlace(Resource):
    @api.expect(place_model)
    @api.response(200, 'Success')
    @api.response(400, 'Invalid request format')
    @api.response(500, 'Internal server error')
    def put(self):
        """Write place information to Firestore"""
        try:
            data = api.payload
            write_place(
                data['address'],
                data['category'],
                data['datePlacesLat'],
                data['datePlacesLon'],
                data['datePlacesName']
            )
            return {"message": "Place information successfully written"}, 200
        except KeyError as e:
            logger.error(f"Missing key in request: {e}")
            api.abort(400, f"Invalid request format: {str(e)}")
        except Exception as e:
            logger.error(f"Error writing place information: {e}")
            api.abort(500, f"Internal server error: {str(e)}")

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
