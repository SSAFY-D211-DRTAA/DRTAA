import boto3
import logging
import time
import os

from dotenv import load_dotenv
from flask import Flask
from flask_cors import CORS
from flask_restx import Api, Resource, fields
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.schema import HumanMessage

from botocore.exceptions import ClientError

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
api = Api(app, version='1.0', title='AI API 문서', description='Swagger 문서', doc="/ai-api-server/api-docs")
llm = ChatOpenAI(api_key=OPENAI_API_KEY, model="gpt-4o-mini")

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

@api.route('/ai-api-server/')
class Home(Resource):
    def get(self):
        """Welcome message"""
        return "Welcome to the API server!"

@api.route('/ai-api-server/process_request')
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

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
