package com.d211.drtaa.global.config.swagger;

import io.swagger.v3.oas.models.Components;
import io.swagger.v3.oas.models.OpenAPI;
import io.swagger.v3.oas.models.info.Info;
import io.swagger.v3.oas.models.security.SecurityScheme;
import io.swagger.v3.oas.models.security.SecurityRequirement;
import io.swagger.v3.oas.models.servers.Server;
import org.springdoc.core.models.GroupedOpenApi;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class SwaggerConfig {
    @Bean
    public OpenAPI openAPI() {
        // Define the security scheme
        SecurityScheme securityScheme = new SecurityScheme()
                .type(SecurityScheme.Type.HTTP)
                .scheme("bearer")
                .bearerFormat("JWT")
                .in(SecurityScheme.In.HEADER)
                .name("Authorization");

        // Define the security requirement
        SecurityRequirement securityRequirement = new SecurityRequirement()
                .addList("bearerAuth");

        return new OpenAPI()
                .addServersItem(new Server().url("https://j11d211.p.ssafy.io") // 배포한 서버
                        .description("Default Server URL"))
                .addServersItem(new Server().url("http://localhost:8080")
                        .description("Local Development Server"))
                .components(new Components().addSecuritySchemes("bearerAuth", securityScheme))
                .addSecurityItem(securityRequirement)
                .info(apiInfo());
    }

    private Info apiInfo() {
        return new Info()
                .title("Drtaa API 문서")
                .description("Drtaa API에 대한 문서입니다.")
                .version("1.0.0");
    }


    @Bean
    public GroupedOpenApi publicApi() {
        return GroupedOpenApi.builder()
                .group("All")
                .pathsToMatch("/**")
                .build();
    }

    @Bean
    public GroupedOpenApi userApi() {
        return GroupedOpenApi.builder()
                .group("User")
                .pathsToMatch("/user/**")
                .build();
    }

    @Bean
    public GroupedOpenApi rentApi() {
        return GroupedOpenApi.builder()
                .group("Rent")
                .pathsToMatch("/rent/**", "/rent-car/**", "/rent-history/**")
                .build();
    }

    @Bean
    public GroupedOpenApi travelApi() {
        return GroupedOpenApi.builder()
                .group("Travel")
                .pathsToMatch("/travel/**", "travel/blog/**", "travel/weather/**")
                .build();
    }

    @Bean
    public GroupedOpenApi paymentApi() {
        return GroupedOpenApi.builder()
                .group("Payment")
                .pathsToMatch("/payment/**")
                .build();
    }
}
