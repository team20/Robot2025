// How to set up (aka what to do if LEDs are not working):
// #1 Make sure the correct board is selected (Arduino Nano), the right comm is selected (changes per pc)
// #2 LED_PIN (check on the board to see what the LEDs are plugged into and change the constant)
// #3 What the type of LEDs they are, though we usually use Neopixels they might be different, check with electrical,
// 	  the other type is making sure that the pixel type flags are correct (whether the original LEDs are RBG, BRG, GRB, etc)
// #4 Try to change the bootloader (you can find this in the IDE under Tools -> Processor)
// #5 Is the Bitstream right?
// #6 The code itself is likely fine so try not to touch that, if anything change the inline commmands in RobotContainer
// #7 It should tell you if it isn't but just in case make sure your libraries are downloaded and updated

#include <Adafruit_NeoPixel.h>
#include <Wire.h>
// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN 3

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 100

// Declare our NeoPixel strip object:
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed, refer to Adafruit_NeoPixel.h
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

/*
 * for additional flags(in the event the LEDs don't display the color you want, you
 * probably need a different bitstream):
 * NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
 * NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
 * NEO_BRG     Pixels are wired for BRG bitstream (whatever LED Strip 2023 uses)
 * NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
 * NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
 * NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
 */

// Do not use old bootloader

/**
 * @brief Shorthand for creating colors in RGB format
 * @param r Red, 0-255
 * @param g Green, 0-255
 * @param b Blue, 0-255
 * @return The color
 */
uint32_t color(int r, int g, int b) {
	return Adafruit_NeoPixel::Color(r, g, b);
}
// frame variable, changes from loop
int colorIndex = 0;
// pattern led strips are on, read in from master/robot
int pattern = 1;
uint32_t teamColor = color(0, 255, 0);
uint32_t noteColor = color(255, 77, 0);
uint32_t offColor = color(0, 0, 0);

/// @brief An array of 6 colors in rainbow order(ROY G BV)
uint32_t RainbowColor[] = {
    color(255, 0, 0),
    color(255, 165, 0),
    color(255, 255, 0),
    color(0, 255, 0),
    color(0, 0, 255),
    color(148, 0, 211)};

void setup() {
	strip.begin();

	strip.setBrightness(200);  // Set BRIGHTNESS to about 1/5 (max = 255)

	Serial.begin(9600);
	Wire.begin(0x18);
	Wire.onReceive(receiveEvent);
}

void loop() {
	switch (pattern) {  // sets up lights to patterns
		                // note: every function returns a color based on colorIndex, the pixel index, and optional color parameters.
		                // the for loops set the pixels to have their corresponding colors based on the pattern function on the colorIndex frame
		case 0:         // reset code
			colorIndex = 0;
			pattern = -1;
		case 1:  // RainbowPartyFunTime!!
			for (int i = 0; i < LED_COUNT; i++) {
				strip.setPixelColor(i, RainbowPartyFunTime(colorIndex, i));
			}
			delay(75);
			break;
		case 2:  // Climb
			for (int i = 0; i < LED_COUNT; i++) {
				strip.setPixelColor(i, BlinkingLights(colorIndex, color(255, 255, 255), color(0, 0, 0)));
			}
			delay(100);
			break;
		case 3:  // Algae
			for (int i = 0; i < LED_COUNT; i++) {
				strip.setPixelColor(i, BlinkingLights(colorIndex, color(0, 255, 255), color(0, 0, 0)));
			}
			delay(100);
			break;
		default:  // display team color
			for (int i = 0; i < LED_COUNT; i++) {
				strip.setPixelColor(i, teamColor);
			}
			delay(150);
			break;
	}
	strip.show();
	colorIndex++;  // next frame
}

void receiveEvent(int bytes) {
	byte x = Wire.read();
	pattern = x;
}

void serialEvent() {
	pattern = Serial.read();
}

/**
 * @brief Makes the LED strip display all the colors of the rainbow while the rainbow moves
 * @param c The colorIndex
 * @param i LED number
 * @return The color
 */
uint32_t RainbowPartyFunTime(int c, int i) {
	return RainbowColor[(c + i) % 6];
}

/**
 * @brief Makes individual LEDs switch back and forth between two colors
 * @param c Color index
 * @param i LED number
 * @param color1 A color
 * @param color2 The other color
 * @return The color that the LED should show
 */
uint32_t TheaterLights(int c, int i, uint32_t color1, uint32_t color2) {  // pixel on color 1 or 2 depending on frame
	if (i % 2 == c % 2) {
		return color1;
	}
	return color2;
}

/**
 * @brief Makes LEDs switch back and forth between colors,
 * using color index will make the whole strip alternate colors,
 * using LED number will make even numbered LEDs one color, and odd numbered LEDs another color
 * @param x A number determining the color to use
 * @param color3 A color
 * @param color4 The other color
 * @return The color for the LED
 */
uint32_t BlinkingLights(int x, uint32_t color3, uint32_t color4) {
	if (x % 2 == 0) {
		return color3;
	}
	return color4;
}
