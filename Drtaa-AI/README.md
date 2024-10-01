# DRTAA - AI

## Nginx

### Setting

- `/etc/nginx/sites-available/drtaa`
    ```
    server {
        listen 80;
        server_name j11d211.p.ssafy.io;
        return 301 https://$server_name$request_uri;
    }

    server {
        listen 443 ssl;
        server_name j11d211.p.ssafy.io;

        ssl_certificate /etc/letsencrypt/live/p.ssafy.io/fullchain.pem;
        ssl_certificate_key /etc/letsencrypt/live/p.ssafy.io/privkey.pem;

        access_log /var/log/nginx/proxy/access.log;
        error_log /var/log/nginx/proxy/error.log;

        location / {
            include /etc/nginx/proxy_params;
            proxy_pass http://localhost:8080;
        }

        location /ai-api-server/ {
            proxy_pass http://localhost:5000;
            proxy_set_header Host $host;
            proxy_set_header X-Real-IP $remote_addr;
            proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
            proxy_set_header X-Forwarded-Proto $scheme;
            proxy_set_header X-Forwarded-Host $server_name;
        }

        location /swaggerui/ {
        proxy_pass http://localhost:5000/swaggerui/;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        }
    }
    ```