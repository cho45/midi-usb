#include "mbed.h"
#include "config.h"
#include "USBMIDI.h"
#include "mcp23017.h"


class QEI {
public:
	int32_t value;
	int8_t last_valid;

	static const int8_t INVALID = 2;
	static uint8_t decode(const uint8_t prev, const uint8_t curr) {
		/**
		 * 4bit decode table
		 *       bit3      bit2      bit1     bit0
		 * [ prev A ][ prev B ][ curr A ][ curr B]
		 *
		 */
		switch ( (prev << 2) | curr) {
			case 0b0000: return  0;
			case 0b0001: return  1;
			case 0b0010: return -1;
			case 0b0011: return  INVALID;
			case 0b0100: return -1;
			case 0b0101: return  0;
			case 0b0110: return  INVALID;
			case 0b0111: return  1;
			case 0b1000: return  1;
			case 0b1001: return  INVALID;
			case 0b1010: return  0;
			case 0b1011: return -1;
			case 0b1100: return  INVALID;
			case 0b1101: return -1;
			case 0b1110: return  1;
			case 0b1111: return  0;
		}
		return INVALID;
	}

	QEI() :
	value(0),
	last_valid(0)
	{
	}

	int8_t sample(uint8_t prev, uint8_t curr) {
		int8_t incr = decode(prev, curr);

		if (incr == INVALID) {
			value += last_valid;
			incr = last_valid;
		} else {
			value += incr;
			last_valid = incr;
		}

		return incr;
	}

	void reset() {
		value = 0;
	}
};

class MatrixController {
	I2C& i2c;
	MCP23017 gpio1;
	bool gpio1_ready;

	static const uint8_t GPIO1_SLAVE_ADDRESS = 0b0100001;

	/**
	 * COL=GPIOA (output normaly positive)
	 * ROW=GPIOB (input pulled-up)
	 */

	static const uint8_t IOCON_VALUE =
			0<<MCP23017::BANK |
			1<<MCP23017::MIRROR |
			1<<MCP23017::SEQOP |
			0<<MCP23017::DISSLW |
			1<<MCP23017::ODR // int pin is open drain
			;


	bool setupGpio(MCP23017& gpio) {
		int ok;
		DEBUG_PRINTF("SET IOCON\r\n");
		ok = gpio.write8(
			MCP23017::IOCON,
			IOCON_VALUE
		);
		if (!ok) return false;

		// IODIR
		//   1: input
		//   0: output
		DEBUG_PRINTF("SET IODIRA\r\n");
		ok = gpio.write16(
			MCP23017::IODIRA,
			0b0000000011111111
		);
		if (!ok) return false;

		// INPUT POLARITY
		//   1: inverse polarity
		//   0: raw
		DEBUG_PRINTF("SET IPOLB\r\n");
		ok = gpio.write8(
			MCP23017::IPOLB,
			0b11111111
		);
		if (!ok) return false;

		// INTERRUPT-ON-CHANGE Enable
		DEBUG_PRINTF("SET GPINTENB\r\n");
		ok = gpio.write8(
			MCP23017::GPINTENB,
			0b11111111
		);
		if (!ok) return false;

		// INTERRUPT-ON-CHANGE Control
		//   1: compared with DEFVAL
		//   0: compared to previous value
		DEBUG_PRINTF("SET INTCONB\r\n");
		ok = gpio.write8(
			MCP23017::INTCONB,
			0b00000000
		);
		if (!ok) return false;

		// PULL-UP (for input pin)
		//   1: pull-up enabled
		//   0: pull-up disabled
		DEBUG_PRINTF("SET GPPUB\r\n");
		ok = gpio.write8(
			MCP23017::GPPUB,
			0b11111111
		);
		if (!ok) return false;

		DEBUG_PRINTF("SET GPIOA\r\n");
		ok = gpio.write8(
			MCP23017::GPIOA,
			0b00000000
		);
		if (!ok) return false;

		return true;
	}

	bool checkGpio(MCP23017& gpio) {
		int ok;
		uint8_t read = gpio.read8(MCP23017::IOCON, ok);
		return read == IOCON_VALUE;
	}

public:

	MatrixController(I2C& _i2c) :
		i2c(_i2c),
		gpio1(i2c, GPIO1_SLAVE_ADDRESS)
	{
	}

	void init() {
		DEBUG_PRINTF("init gpio1\r\n");
		gpio1_ready = setupGpio(gpio1);
		DEBUG_PRINTF("gpio1 initialized: %s\r\n", gpio1_ready ? "success" : "failed");
	}

	void scanMatrix(uint8_t *keys) {
		int ok;

		for (int i = 0; i < 8; i++) {
			ok = gpio1.write8(
				MCP23017::GPIOA,
				~(1<<i)
			);
			wait_us(1);
			keys[i] = gpio1.read8(MCP23017::GPIOB, ok);
		}
		ok = gpio1.write8(
			MCP23017::GPIOA,
			0b00000000
		);
	}
};

void show_message(MIDIMessage msg) {
	switch (msg.type()) {
		case MIDIMessage::NoteOnType:
			printf("NoteOn key:%d, velocity: %d, channel: %d\r\n", msg.key(), msg.velocity(), msg.channel());
			break;
		case MIDIMessage::NoteOffType:
			printf("NoteOff key:%d, velocity: %d, channel: %d\r\n", msg.key(), msg.velocity(), msg.channel());
			break;
		case MIDIMessage::ControlChangeType:
			printf("ControlChange controller: %d, data: %d\r\n", msg.controller(), msg.value());
			break;
		case MIDIMessage::PitchWheelType:
			printf("PitchWheel channel: %d, pitch: %d\r\n", msg.channel(), msg.pitch());
			break;
		default:
			printf("Another message\r\n");
	}
}


USBMIDI midi;
DigitalOut led0(P0_13);
DigitalOut led1(P0_11);
DigitalOut led2(P0_12);
static I2C i2c(P0_5, P0_4);
static MatrixController matrixController(i2c);

static const uint8_t ENCODER_NUMS = 19;
static const uint8_t COLS = 8;
static uint8_t keysA[COLS];
static uint8_t keysB[COLS];
static bool state = 0;
static uint8_t layer = 0;
static QEI encoders[ENCODER_NUMS];

struct colrow {
	uint8_t col;
	uint8_t row;
};

static const colrow buttonMap[26] = {
	{7, 0}, // 1
	{7, 1}, // 2
	{7, 2}, // 3
	{7, 3}, // 4
	{7, 4}, // 5
	{7, 5}, // 6
	{7, 6}, // 7
	{7, 7}, // 8

	{6, 0}, // 9
	{6, 1}, // 10
	{6, 2}, // 11
	{6, 3}, // 12
	{6, 4}, // 13
	{6, 5}, // 14
	{6, 6}, // 15
	{6, 7}, // 16

	{5, 0}, // 17
	{5, 1}, // 18
	{5, 2}, // 19
	{5, 3}, // 20
	{5, 6}, // 21
	{5, 4}, // 22
	{5, 7}, // 23
	{5, 5}, // 24

	{4, 6}, // 25
	{4, 7}, // 26
};

static const colrow encoderMap[19] = {
	{0, 0}, // 1
	{0, 2}, // 2
	{0, 4}, // 3
	{0, 6}, // 4
	{1, 0}, // 5
	{1, 2}, // 6
	{1, 4}, // 7
	{1, 6}, // 8
	{2, 0}, // 9
	{2, 2}, // 10
	{2, 4}, // 11
	{2, 6}, // 12
	{3, 0}, // 13
	{3, 2}, // 14
	{3, 4}, // 15
	{3, 6}, // 11
	{4, 0}, // 17
	{4, 2}, // 18
	{4, 4}, // 19
};

int main() {
	// 100k
	// i2c.frequency(100000);
	// 400k (max @3.3V for MCP23017)
	i2c.frequency(400000);

	printf("init\r\n");

	matrixController.init();

	midi.attach(show_message);
	led0 = 1; led1 = 0; led2 = 0;

	while (1) {
		uint8_t (&keysCurr)[COLS] = state ? keysA : keysB;
		uint8_t (&keysPrev)[COLS] = state ? keysB : keysA;

		/* 
		 *	keys[0]	keys[1]	keys[2]	keys[3]	keys[4]	keys[5]	keys[6]	keys[7]
		 *	0	r1 A	r5 A	r9 A	r13 A	r17 A	r17 sw	r9 sw	r1 sw
		 *	1	r1 B	r5 B	r9 B	r13 B	r17 B	r18 sw	r10 sw	r2 sw
		 *	2	r2 A	r6 A	r10 A	r14 A	r18 A	r19 sw	r11 sw	r3 sw
		 *	3	r2 B	r6 B	r10 B	r14 B	r18 B	sw20	r12 sw	r4 sw
		 *	4	r3 A	r7 A	r11 A	r15 A	r19 A	sw22	r13 sw	r5 sw
		 *	5	r3 B	r7 B	r11 B	r15 B	r19 B	sw24	r14 sw	r6 sw
		 *	6	r4 A	r8 A	r12 A	r16 A	sw25	sw21	r15 sw	r7 sw
		 *	7	r4 B	r8 B	r12 B	r16 B	sw26	sw23	r16 sw	r8 sw
		 *
		 *	sw20 sw22 sw24 -> layer shift
		 */
		matrixController.scanMatrix(keysCurr);

		for (int num = 0; num < ENCODER_NUMS; num++) {
			const colrow e = encoderMap[num];
			QEI& encoder = encoders[num];
			const int8_t sampled = encoder.sample(
				((keysPrev[e.col] >> e.row) & 0b11),
				(keysCurr[e.col] >> e.row & 0b11)
			);

			if (sampled) {
				DEBUG_PRINTF("(%d)+=%d v=%d\r\n", num+1, sampled, encoder.value);
			}

			if (abs(encoder.value) >= 4) {
				// changed
				const uint8_t control = num + 1 + (layer * 20);
				const int8_t value = encoder.value / 4;
				DEBUG_PRINTF("CC %d %d\r\n", control, value);
				midi.write(MIDIMessage::ControlChange(control, value, 0));
				encoder.reset();
			}
		}

		for (int num = 0; num < 26; num++) {
			const colrow b = buttonMap[num];
			const uint8_t prev = (keysPrev[b.col] >> b.row) & 1;
			const uint8_t curr = (keysCurr[b.col] >> b.row) & 1;
			const uint8_t button = num + 1;
			if (prev != curr) {
				switch (button) {
					case 20:
						DEBUG_PRINTF("SWITCH LAYER TO 0\r\n");
						led0 = 1; led1 = 0; led2 = 0;
						layer = 0; break;
					case 22:
						DEBUG_PRINTF("SWITCH LAYER TO 1\r\n");
						led0 = 0; led1 = 1; led2 = 0;
						layer = 1; break;
					case 24:
						DEBUG_PRINTF("SWITCH LAYER TO 2\r\n");
						led0 = 0; led1 = 0; led2 = 1;
						layer = 2; break;
					default:
						if (curr) {
							DEBUG_PRINTF("Note On %d\r\n", button);
							midi.write(MIDIMessage::NoteOn(button));
						} else {
							DEBUG_PRINTF("Note Off %d\r\n", button);
							midi.write(MIDIMessage::NoteOff(button));
						}
				}
			}
		}

		state = !state;
		wait_ms(1);
	}
}

