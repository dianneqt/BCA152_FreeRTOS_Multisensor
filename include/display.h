#pragma once

// DisplayTask: the ONLY task that touches the OLED (SSD1306 over I2C).
void displayTask(void *parameter);