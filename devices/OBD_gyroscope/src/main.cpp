//#include <Arduino.h>
//#include <FreematicsPlus.h>
//
//#define STATE_MEMS_READY 0x8
//
//#define ENABLE_MEMS true
//
//#if ENABLE_MEMS
//float accBias[3] = {0};
//float accSum[3] = {0};
//float acc[3] = {0};
//float gyr[3] = {0};
//float mag[3] = {0};
//uint8_t accCount = 0;
//#endif
//
//MEMS_I2C* mems = 0;
//
//class State {
//	public:
//		bool check(uint16_t flags){return (m_state & flags) == flags; }
//		void set(uint16_t flags) { m_state |= flags; }
//		void clear(uint16_t flags) { m_state &= ~flags; }
//		uint16_t m_state = 0;
//};
//
//State state;

//void setup(){
//	if(!state.check(STATE_MEMS_READY)) do {
//		Serial.print("MEMS: ");
//		mems = new ICM_42627;
//		byte ret = mems->begin();
//		if(ret){
//			state.set(STATE_MEMS_READY);
//			Serial.println("ICM_42627");
//			break;
//		}
//		delete mems;
//		mems = 0;
//		Serial.println("NO");
//	} while(0);
//}

//void loop(){
//	if(!state.check(STATE_MEMS_READY)) do {
//		Serial.print("MEMS: ");
//		mems = new ICM_42627;
//		byte ret = mems->begin();
//		if(ret){
//			state.set(STATE_MEMS_READY);
//			Serial.println("ICM_42627");
//			break;
//		}
//		delete mems;
//		mems = 0;
//		Serial.println("NO");
//	} while(0);
//}

#include <Arduino.h>
#include <FreematicsPlus.h>

#define STATE_MEMS_READY 0x8
#define ENABLE_MEMS true

#if ENABLE_MEMS
float accBias[3] = {0};
float accSum[3] = {0};
float acc[3] = {0}; // Acelerômetro (x, y, z)
float gyr[3] = {0}; // Giroscópio (x, y, z)
float mag[3] = {0}; // Magnetômetro (x, y, z)
uint8_t accCount = 0;
#endif

MEMS_I2C* mems = 0;

class State {
public:
    bool check(uint16_t flags) { return (m_state & flags) == flags; }
    void set(uint16_t flags) { m_state |= flags; }
    void clear(uint16_t flags) { m_state &= ~flags; }
    uint16_t m_state = 0;
};

State state;

void setup() {
    Serial.begin(115200); // Inicializa a comunicação serial
    delay(1000); // Aguarda a estabilização da serial

    // Inicialização do sensor
    if (!state.check(STATE_MEMS_READY)) {
        Serial.print("MEMS: ");
        mems = new ICM_42627;
        byte ret = mems->begin();
        if (ret) {
            state.set(STATE_MEMS_READY);
            Serial.println("ICM_42627 inicializado com sucesso");
        } else {
            Serial.println("Falha ao inicializar ICM_42627");
            delete mems;
            mems = 0;
        }
    }
}

void loop() {
    if (state.check(STATE_MEMS_READY)) {
        // Ler dados do sensor
        if (mems->read(acc, gyr, mag)) { // Verifica se a leitura foi bem-sucedida
            // Exibir dados do giroscópio
            Serial.print("Giroscópio (deg/s): ");
            Serial.print("X=");
            Serial.print(gyr[0]);
            Serial.print(" Y=");
            Serial.print(gyr[1]);
            Serial.print(" Z=");
            Serial.println(gyr[2]);

            // Opcional: Exibir dados do acelerômetro
            Serial.print("Acelerômetro (m/s²): ");
            Serial.print("X=");
            Serial.print(acc[0]);
            Serial.print(" Y=");
            Serial.print(acc[1]);
            Serial.print(" Z=");
            Serial.println(acc[2]);

            // Opcional: Exibir dados do magnetômetro (se disponível)
            Serial.print("Magnetômetro (uT): ");
            Serial.print("X=");
            Serial.print(mag[0]);
            Serial.print(" Y=");
            Serial.print(mag[1]);
            Serial.print(" Z=");
            Serial.println(mag[2]);
        } else {
            Serial.println("Falha ao ler os dados do sensor");
        }
    } else {
        Serial.println("Sensor não inicializado");
    }

    delay(100); // Ajuste o intervalo de leitura conforme necessário
}