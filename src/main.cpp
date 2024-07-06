/*
  Motus Test Tag Simulater
  written by Thorsten von Eicken and William Dittmann

  base hardware: Adafruit Feather M0 RFM96 LoRa Radio - 433MHz - RadioFruit Product ID: 3179
  antenna used: TWAYRDIO 15.4-Inch Whip Antenna, SMA Male Antenna, Dual Band 2m/70cm 
              although smaller dual band antenna would be more appropriate for a deployed model
    
  this code will alternatively transmit a CTT test tag followed by 6 pulses of a lotek tag to facilitate 
  verification of the working status of a sensor gnome station at a more cost friendly price point.
  tag values should be set like this at around line 58

    // lotek TAG to transmit to sensorGnome
  static uint16_t tagCode[] = {22, 54, 29, 81}; // ms, ms, ms, 1/10th sec
    // ctt tag should be 4 bytes, DO NOT INCLUDE THE CRC BYTE, The CRC byte is calculated
  static uint16_t cttTag[] = { 0x78, 0x66, 0x4C, 0x33 }; 

 

  Other modules from SX127x/RFM9x family can also be used.

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>  // https://github.com/jgromes/RadioLib
#include <CRC.h> // https://github.com/RobTillaart/CRC


#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif

// SX1278 has the following connections:
// NSS pin:   8
// DIO0 pin:  3
// RESET pin: 4
// DIO1 pin:  3
SX1278 radio = new Module(8, 3, 4, 3);

// or using RadioShield
// https://github.com/jgromes/RadioShield
// SX1278 radio = RadioShield.ModuleA;

// save transmission state between loops
int cttTransmissionState = RADIOLIB_ERR_NONE, lotekTransmissionState = RADIOLIB_ERR_NONE;

#define MAXPKT 200
static uint8_t packet[MAXPKT];
static uint8_t packetLen = 0;
#define BR 2 // bit rate in kbps
uint8_t sync[] = {0xD3, 0x91}; // sync word syncword for CTT Tags


// lotek TAG to transmit to sensorGnome
//static uint16_t tagCode[] = {22, 54, 29, 81}; // ms, ms, ms, 1/10th sec
//static uint16_t tagCode[] = {22, 20, 24, 200}; // ms, ms, ms, 1/10th sec
static uint16_t tagCode[] = {22, 24, 44, 200}; // ms, ms, ms, 1/10th sec
// ctt tag should be 4 bytes, DO NOT INCLUDE THE CRC BYTE, The CRC byte is calculated
static uint16_t cttTag[] = { 0x78, 0x66, 0x4C, 0x33 }; 


// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;
// bool transmittedFlag = false;  // flag that a packet was transmitted

// counter to keep track of transmitted packets
int count = 0;

void setFlag(void)
{
  // we sent a packet, set the flag
  transmittedFlag = true;
}

int configCTT()
{
  int state = -1;

Serial.print("reconfigure radio =>");
  const float FREQ = 434;
  const float cttBR = 25;
  const float FREQDEV = 25;
  const float RXBW = 58.6;
  const float POW = 10;
  const float PRELEN = 32;
  const float ENABLEOOK = false;
  // the following settings can also
  // be modified at run-time
  state = radio.setFrequency(FREQ);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setFrequency, code "));
    Serial.println(state);
  }
  state = radio.setBitRate(cttBR);
     if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setBitRate, code "));
    Serial.println(state);
  }
  state = radio.setFrequencyDeviation(FREQDEV);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setFrequencyDeviation, code "));
    Serial.println(state);
  }
  state = radio.setRxBandwidth(RXBW);
    if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setRxBandwidth, code "));
    Serial.println(state);
  }
  state = radio.setOutputPower(POW);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setOutputPower, code "));
    Serial.println(state);
  }
  state = radio.setCurrentLimit(100);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setCurrentLimit, code "));
    Serial.println(state);
  }
  state = radio.setPreambleLength(PRELEN);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setPreambleLength, code "));
    Serial.println(state);
  }
 state = radio.setDataShaping(RADIOLIB_SHAPING_NONE);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change modulation, code "));
    Serial.println(state);
  }
 state = radio.fixedPacketLengthMode(5);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change fixedPacketLengthMode, code "));
    Serial.println(state);
  }
  state = radio.setCRC(0);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setCRC, code "));
    Serial.println(state);
  }
  state = radio.setSyncWord(sync, sizeof(sync));

  // FSK modulation can be changed to OOK
  // NOTE: When using OOK, the maximum bit rate is only 32.768 kbps!
  //       Also, data shaping changes from Gaussian filter to
  //       simple filter with cutoff frequency. Make sure to call
  //       setDataShapingOOK() to set the correct shaping!
  state = radio.setOOK(ENABLEOOK);
  //  state = radio.setDataShapingOOK(1);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change modulation, code "));
    Serial.println(state);
  }
  // else result = true;

//Serial.print(radio.);
  return state;
}


uint8_t addPulse(uint16_t millis) {
  uint8_t byteOff = (millis*BR)>>3;
  uint8_t bitOff = (millis*BR) & 7;
  for (int i=0; i<5; i++) {
    packet[byteOff] |= 0x80 >> bitOff;
    bitOff++;
    if (bitOff == 8) {
      bitOff = 0;
      byteOff++;
    }
  }
  return byteOff+1;
}

int configlotek()
{
  int state = -1;
  const float lotekFreq = 166.38;
  const float Br = BR;
  const float FREQDEV = 4.0;
  const float RXBW = 9.7;
  const float POW = 10;
  const float PRELEN = 24;

  const bool ENABLEOOK = true;

  // the following settings can also
  // be modified at run-time
  state = radio.setFrequency(lotekFreq);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setFrequency, code "));
    Serial.println(state);
  }
  state = radio.setBitRate(Br);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setBitRate, code "));
    Serial.println(state);
  }
  state = radio.setFrequencyDeviation(FREQDEV);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setFrequencyDeviation, code "));
    Serial.println(state);
  }
  state = radio.setRxBandwidth(RXBW);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setRxBandwidth, code "));
    Serial.println(state);
  }
  state = radio.setOutputPower(POW);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setOutputPower, code "));
    Serial.println(state);
  }
  state = radio.setCurrentLimit(100);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setCurrentLimit, code "));
    Serial.println(state);
  }
  state = radio.setDataShaping(RADIOLIB_SHAPING_NONE);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setDataShaping, code "));
    Serial.println(state);
  }
  state = radio.setPreambleLength(PRELEN);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setPreambleLength, code "));
    Serial.println(state);
  }
  // FSK modulation can be changed to OOK
  // NOTE: When using OOK, the maximum bit rate is only 32.768 kbps!
  //       Also, data shaping changes from Gaussian filter to
  //       simple filter with cutoff frequency. Make sure to call
  //       setDataShapingOOK() to set the correct shaping!
  state = radio.setOOK(ENABLEOOK);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setOOK, code "));
    Serial.println(state);
  }
  state = radio.setSyncWord(nullptr, 0);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setSyncWord, code "));
    Serial.println(state);
  }
  state = radio.setCurrentLimit(100);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setCurrentLimit, code "));
    Serial.println(state);
  }
  state = radio.setDataShapingOOK(1);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setDataShapingOOK, code "));
    Serial.println(state);
  }
  state = radio.setCRC(0); 
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Unable to change setCRC, code "));
    Serial.println(state);
  }

  // Generate packet to match desired burst
  for (int i=0; i<MAXPKT; i++) packet[i] = 0;
  int off = 16; addPulse(off);
  off += tagCode[0];
  for (int i=0; i<3; i++, off+=tagCode[i]) packetLen = addPulse(off);
  radio.fixedPacketLengthMode(packetLen);

  for (int i=0; i<packetLen; i++) {
    Serial.print(" ");
    Serial.print(packet[i], 16);
  }
  Serial.println();

  return state;
}



void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.print(F("[SX1278] Initializing ... /n"));

  int state = radio.beginFSK();
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("success!"));
  }
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  // set the function that will be called
  // when packet transmission is finished
  radio.setPacketSentAction(setFlag);

  transmittedFlag = true;
}

void loop()
{
  int state;
  // check if the previous transmission finished
  if (transmittedFlag)
  {
    // reset flag
    transmittedFlag = false;

    if (cttTransmissionState == RADIOLIB_ERR_NONE)
    {
      // packet was successfully sent
      Serial.println(F("transmission finished!"));

      // NOTE: when using interrupt-driven transmit method,
      //       it is not possible to automatically measure
      //       transmission data rate using getDataRate()
    }
    else
    {
      Serial.print(F("failed, code "));
      Serial.println(cttTransmissionState);
    }

    // clean up after transmission is finished
    // this will ensure transmitter is disabled,
    // RF switch is powered down etc.
    state = radio.finishTransmit();
    
    // prep packet
    if ((count % 2) ==  0)
      {
      // clean up after transmission is finished
      // this will ensure transmitter is disabled,
      // RF switch is powered down etc.
      state = radio.finishTransmit();
      if (state != RADIOLIB_ERR_NONE)
      {
        Serial.print(F("Unable to set finish xmit, code "));
        Serial.println(state);
      }

      for (int i=0; i<MAXPKT; i++) packet[i] = 0;
      // tag 0x 78664c33 (58)
      packet[0] = cttTag[0];
      packet[1] = cttTag[1];
      packet[2] = cttTag[2];
      packet[3] = cttTag[3];
      packet[4] = calcCRC8(packet, 4);
      packetLen = 5;

      int state = configCTT();

      if (state != RADIOLIB_ERR_NONE)
      {
        Serial.print(F("Unable to set configuration, code "));
        Serial.println(state);
      }
      Serial.printf("Packet: %06d  %02X %02X %02X %02X  CRC:%02X\n",
                    count++, packet[0], packet[1], packet[2], packet[3], packet[4]);

      // transmit the packet and activate the LED
      digitalWrite(LED_BUILTIN, HIGH); // while (true);
      cttTransmissionState = radio.startTransmit(packet, packetLen);
      digitalWrite(LED_BUILTIN, LOW);

      // wait a second before transmitting again
      delay(5000);
    }
    else
    {
      Serial.printf("\n\n\n\nMostag Lotek test [%d %d %d %d.%ds]\n",
                    tagCode[0], tagCode[1], tagCode[2], tagCode[3] / 10, tagCode[3] % 10);
      int state = configlotek();
      if (state != RADIOLIB_ERR_NONE)
      {
        Serial.print(F("Unable to change modulation, code "));
        Serial.println(state);
      }

      uint32_t next = 0;

      for (int ii=0; ii<6; ii++) {
        if (next-millis() < 120000) delay(next-millis());
        next = millis()+tagCode[3]*100;

        digitalWrite(LED_BUILTIN, HIGH);
        state = radio.transmit(packet, packetLen);
        digitalWrite(LED_BUILTIN, LOW);
        if (state != RADIOLIB_ERR_NONE)
        {
          Serial.print(F("Unable to change modulation, code "));
          Serial.println(state);
        }

        Serial.print("Lotek TX [");
        Serial.print(tagCode[0]); Serial.print(" ");
        Serial.print(tagCode[1]); Serial.print(" ");
        Serial.print(tagCode[2]); Serial.print("] ");
        Serial.print(tagCode[3]/10); Serial.print(".");
        Serial.print(tagCode[3]%10); Serial.print("ms #");
        Serial.println(ii);

      }
      state = radio.sleep();
      if (state != RADIOLIB_ERR_NONE)
      {
        Serial.print(F("Unable to change modulation, code "));
        Serial.println(state);
      }

      radio.finishTransmit();
      if (state != RADIOLIB_ERR_NONE)
      {
        Serial.print(F("Unable to change modulation, code "));
        Serial.println(state);
      }

      count++;
      delay(1000);

    }
  }
}