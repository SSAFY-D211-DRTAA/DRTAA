FROM python:3.12-slim
WORKDIR /app
COPY . .
WORKDIR /app/api_server
RUN pip install --no-cache-dir -r requirements.txt
CMD ["python", "app.py"]
