set(COMPONENT_SRCS "app_main.c")

set(COMPONENT_REQUIRES
    esp32-camera
    nvs_flash
    esp_http_server
    camera
    protocol_examples_common
    )

register_component()
