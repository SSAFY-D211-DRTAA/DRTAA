package com.d211.drtaa.global.config.mqtt;


import lombok.extern.log4j.Log4j2;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.integration.annotation.IntegrationComponentScan;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.integration.mqtt.core.DefaultMqttPahoClientFactory;
import org.springframework.integration.mqtt.core.MqttPahoClientFactory;
import org.springframework.integration.mqtt.outbound.MqttPahoMessageHandler;
import org.springframework.messaging.MessageHandler;

@Configuration
@Log4j2
@IntegrationComponentScan
public class MqttConfig {
    private static final String TAG = MqttConfig.class.getSimpleName();

    @Value("${spring.mqtt.broken-url}")
    private String brokenUrl;

    @Value("${spring.mqtt.client-id}")
    private String cliendId;

    @Value("${spring.mqtt.topic}")
    private String topic;

    @Bean
    public MqttPahoClientFactory mqttPahoClientFactory(){
        DefaultMqttPahoClientFactory factory = new DefaultMqttPahoClientFactory();
        MqttConnectOptions options = new MqttConnectOptions();
        options.setServerURIs(new String[]{brokenUrl});
        factory.setConnectionOptions(options);
        return factory;
    }

    @Bean
    @ServiceActivator(inputChannel = "mqttOutboundChannel")
    public MessageHandler mqttOutbound(){
        MqttPahoMessageHandler messageHandler = new MqttPahoMessageHandler(cliendId,mqttPahoClientFactory());
        //test
    }

}
