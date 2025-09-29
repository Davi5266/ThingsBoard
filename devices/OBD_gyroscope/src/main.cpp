#include <Arduino.h>
#include <FreematicsPlus.h>

#define STATE_MEMS_READY 0x8

#if ENABLE_MEMS
float accBias[3] = {0};
float accSum[3] = {0};
float acc[3] = {0};
float gyr[3] = {0};
float mag[3] = {0};
uint8_t accCount = 0;
#endif

MEMS_I2C* mems = 0;

class State {
	public:
		bool check(uint16_t flags){return (m_state & flags) == flags; }
		void set(uint16_t flags) { m_state |= flags; }
		void clear(uint16_t flags) { m_state &= ~flags; }
		uint16_t m_state = 0;
};

State state;

void setup(){
	if(!state.check(STATE_MEMS_READY)) do {
		Serial.print("MEMS: ");
		mems = new ICM_42627;
		byte ret = mems->begin();
		if(ret){
			state.set(STATE_MEMS_READY);
			Serial.println("ICM_42627");
			break;
		}
		delete mems;
		mems = 0;
		Serial.println("NO");
	} while(0);
}

void loop(){
	if(!state.check(STATE_MEMS_READY)) do {
		Serial.print("MEMS: ");
		mems = new ICM_42627;
		byte ret = mems->begin();
		if(ret){
			state.set(STATE_MEMS_READY);
			Serial.println("ICM_42627");
			break;
		}
		delete mems;
		mems = 0;
		Serial.println("NO");
	} while(0);
}
