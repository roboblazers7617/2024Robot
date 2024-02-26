// LED Arduino code for FRC Team 7167's 2024 season robot
// Could be modified to be generic, just change the color modes under serialEvent

#include <FastLED.h>
#include <Wire.h>

// How many leds in your strip?
#define NUM_LEDS 180

// For led chips like WS2812, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
// Clock pin only needed for SPI based chipsets when not using hardware SPI
#define DATA_PIN 3
#define CLOCK_PIN 13

// Define the array of leds
CRGB leds[NUM_LEDS];

// Global animation settings
CRGB backgroundColor = CRGB::Black;
CRGB foregroundColor = CRGB::Blue;
int animationFrameDelay = 2;

// LED Mode
// Possible modes are:
//  0 - solid (set whole strip to foreground color and never update it)
//  1 - blink
//  2 - bounce
//  3 - rainbow
int mode = 3;

// Frame number to help with frame spacing
int frameNumber;

// Frames since last heartbeat
#define FRAMES_UNTIL_HEARTBEAT_DEAD 10
int framesSinceHeartbeat = 0;

// String for serial data
String inputString = "";

// Bounce animation settings
int bouncePosition = 0;
int bounceLength = 10;
bool bounceDirection = true;

// Blink animation settings
bool blinkState = true;

// Rainbow animation things
bool rainbowFrame = 0;

// Boolean to tell whether a solid color has been set to prevent running drawSolidColor() multiple times
bool solidColorSet = true;

void setup() {
	// Uncomment/edit one of the following lines for your leds arrangement.
	// ## Clockless types ##
	FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
	// FastLED.addLeds<SM16703, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<TM1829, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<TM1812, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<TM1809, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<TM1804, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<TM1803, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<UCS1903, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<UCS1903B, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<UCS1904, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<UCS2903, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB ordering is typical
	// FastLED.addLeds<WS2852, DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB ordering is typical
	// FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB ordering is typical
	// FastLED.addLeds<GS1903, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<SK6812, DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB ordering is typical
	// FastLED.addLeds<SK6822, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<APA106, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<PL9823, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<SK6822, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<WS2813, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<APA104, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<WS2811_400, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<GE8822, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<GW6205, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<GW6205_400, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<LPD1886, DATA_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<LPD1886_8BIT, DATA_PIN, RGB>(leds, NUM_LEDS);
	// ## Clocked (SPI) types ##
	// FastLED.addLeds<LPD6803, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);  // GRB ordering is typical
	// FastLED.addLeds<LPD8806, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);  // GRB ordering is typical
	// FastLED.addLeds<WS2801, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<WS2803, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<SM16716, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
	// FastLED.addLeds<P9813, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);  // BGR ordering is typical
	// FastLED.addLeds<DOTSTAR, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);  // BGR ordering is typical
	// FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);  // BGR ordering is typical
	// FastLED.addLeds<SK9822, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);  // BGR ordering is typical

	Serial.begin(9600);
	Serial1.begin(9600);
	inputString.reserve(200);
}

// Loops through every LED and sets them to a color
void drawSolidColor(CRGB color) {
	for (int i = 0; i <= NUM_LEDS; i++) {
		leds[i] = color;
	}
}

// Loops through every LED and sets a range to foregroundColor and the rest to backgroundColor
void drawSegment(int segmentLength, int position, CRGB backgroundColor, CRGB foregroundColor) {
	for (int i = 0; i <= NUM_LEDS; i++) {
		// If within segment, set to foregroundColor
		if (i >= position && i <= position + segmentLength) {
			leds[i] = foregroundColor;
		}
		// Otherwise, set to backgroundColor
		else {
			leds[i] = backgroundColor;
		}
	}
}

// Toggles the whole strip between backgroundColor and foregroundColor
void blinkAnimation() {
	if (blinkState) {
		drawSolidColor(foregroundColor);
	} else {
		drawSolidColor(backgroundColor);
	}

	blinkState = !blinkState;
}

void bounceAnimation() {
	drawSegment(bounceLength, bouncePosition, backgroundColor, foregroundColor);

	// Number of possible positions for the bounce to be in
	int bouncePositions = NUM_LEDS - bounceLength;

	if (bounceDirection) {
		bouncePosition++;
	} else {
		bouncePosition--;
	}

	//Serial.println(bouncePosition);

	if (bouncePosition <= 0 || bouncePosition >= bouncePositions) {
		bounceDirection = !bounceDirection;
	}
}

void rainbowAnimation() {
	for (int i = 0; i < NUM_LEDS; i++) {
		leds[i] = CHSV(i - (frameNumber * 2), 255, 255); /* The higher the value 4 the less fade there is and vice versa */
	}
}

void updateLEDS() {
	if ((frameNumber % animationFrameDelay) == 0) {
		switch (mode) {
		case 0:
			if (!solidColorSet) {
				drawSolidColor(foregroundColor);
				FastLED.show();
				solidColorSet = true;
			}
			break;
		case 1:
			blinkAnimation();
			FastLED.show();
			break;
		case 2:
			bounceAnimation();
			FastLED.show();
			break;
		case 3:
			rainbowAnimation();
			FastLED.show();
			break;
		}
	}

	frameNumber++;
	framesSinceHeartbeat++;
}

void loop() {
	while (Serial1.available()) {
		framesSinceHeartbeat = 0;
		// Get new byte
		char inChar = (char)Serial1.read();

		// Do things on newline
		if (inChar == '\n') {
			// Set bounce to start at the start if the mode is not already bounce
			if (inputString == "dp" || inputString == "es" || inputString == "td" || inputString == "ad") {
				if (mode != 2) {
					bouncePosition = 0;
					bounceDirection = true;
				}
			}

			// DS Disconnected
			if (inputString == "dp") {
				backgroundColor = CRGB::Black;
				foregroundColor = CRGB::Blue;
				mode = 2;
				animationFrameDelay = 2;
			}

			// E-stop
			else if (inputString == "es") {
				backgroundColor = CRGB::Red;
				foregroundColor = CRGB::Yellow;
				mode = 2;
				animationFrameDelay = 2;
			}

			// Teleop disabled
			else if (inputString == "td") {
				backgroundColor = CRGB::Black;
				foregroundColor = CRGB::Purple;
				mode = 2;
				animationFrameDelay = 2;
			}

			// Teleop enabled
			else if (inputString == "te") {
				foregroundColor = CRGB::Purple;
				mode = 0;
				solidColorSet = false;
			}

			// Auto disabled
			else if (inputString == "ad") {
				backgroundColor = CRGB::Black;
				foregroundColor = CRGB::Red;
				mode = 2;
				animationFrameDelay = 2;
			}

			// Auto enabled
			else if (inputString == "ae") {
				foregroundColor = CRGB::Red;
				mode = 0;
				solidColorSet = false;
			}

			// Holding Note
			else if (inputString == "hn") {
				foregroundColor = CRGB::Yellow;
				mode = 0;
				solidColorSet = false;
			}

			// Ready to shoot
			else if (inputString == "rs") {
				foregroundColor = CRGB::Green;
				mode = 0;
				solidColorSet = false;
			}

			// RAINBOW
			else if (inputString == "rb") {
				animationFrameDelay = 2;
				mode = 3;
			}

			inputString = "";
		} else {
			// Add the new byte to the inputString
			inputString += inChar;
		}
	}

	// If there has been no heartbeat for a while, set the LEDs to the idle animation
	if (framesSinceHeartbeat >= FRAMES_UNTIL_HEARTBEAT_DEAD) {
		animationFrameDelay = 1;
		mode = 3;
	}

	EVERY_N_MILLISECONDS(15) { updateLEDS(); }
}