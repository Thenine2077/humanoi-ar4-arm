// ===== AR4_nano_inputs — copy of AR4_nano_sketch.ino with input pins D3-D7 enabled =====
// Only differences from the original: the const Input4-7 lines and pinMode(Input3-7, INPUT_PULLUP)
// that the vendor had commented out are enabled (the original is untouched)
// Pins D3-D7 read HIGH when idle (pull-up) and LOW when a relay contact/switch connects them to GND
//   -> in the HMI use "Wait Input Off = 3" to wait for the relay to turn "on"
// D2 = gripper servo (not used as an input), D14-D19 (A0-A5) = outputs to the relay module
//
// v2: the 8-channel relay module in the box is active-LOW (pin LOW = relay on), confirmed on the real hardware
//     (relay channels 3-8 wired to D14-D19, channels 1-2 not wired), so the original had 2 problems:
//       - the pins are LOW at boot -> all 6 relays "switch on by themselves" as soon as power comes on
//       - the 打开IO (ON) button turns the relay "off" — the opposite of the button name
//     Fix: set pins 14-19 HIGH from boot (relays off) and invert the ON/OF logic for pins 14-19 only
//     -> 打开IO = relay on, 关闭IO = relay off, power loss/Nano reset = relay always off
//     Other pins work as in the original (so ONX3 can't disable an input pin's pull-up)


#include <Servo.h>



String inData;


Servo servo0;
//Servo servo1;
//Servo servo2;
//Servo servo3;
//Servo servo4;
//Servo servo5;
//Servo servo6;
//Servo servo7;


//const int Input2 = 2;
const int Input3 = 3;
const int Input4 = 4;
const int Input5 = 5;
const int Input6 = 6;
const int Input7 = 7;

const int Output14 = 14;
const int Output15 = 15;
const int Output16 = 16;
const int Output17 = 17;
const int Output18 = 18;
const int Output19 = 19;


// v2: pins wired to the relay module (active-LOW)
bool isRelayPin(int pin) {
  return pin >= Output14 && pin <= Output19;
}

void setup() {
  // run once:
  Serial.begin(115200);
//
//  pinMode(A0, OUTPUT);
//  pinMode(A1, OUTPUT);
  pinMode(2, OUTPUT);
//  pinMode(A3, OUTPUT);
//  pinMode(A4, OUTPUT);
//  pinMode(A5, OUTPUT);
//  pinMode(A6, OUTPUT);
//  pinMode(A7, OUTPUT);
//
//
//  pinMode(Input2, OUTPUT);
  pinMode(Input3, INPUT_PULLUP);
  pinMode(Input4, INPUT_PULLUP);
  pinMode(Input5, INPUT_PULLUP);
  pinMode(Input6, INPUT_PULLUP);
  pinMode(Input7, INPUT_PULLUP);


  // v2: write HIGH "before" switching to OUTPUT — the pin outputs HIGH the moment it becomes an output
  // with no brief LOW blip that would make the (active-LOW) relays twitch at boot
  for (int p = Output14; p <= Output19; p++) {
    digitalWrite(p, HIGH);
    pinMode(p, OUTPUT);
  }


  servo0.attach(2);
//  servo1.attach(A1);
//  servo2.attach(A2);
//  servo3.attach(A3);
//  servo4.attach(A4);
//  servo5.attach(A5);
//  servo6.attach(A6);
//  servo7.attach(A7);





}

void loop() {
  //start loop
  while (Serial.available() > 0)
  {
    char recieved = Serial.read();
    inData += recieved;
    // Process message when new line character is recieved
    if (recieved == '\n')
    {
      String function = inData.substring(0, 2);


      //-----COMMAND TO MOVE SERVO---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "SV")
      {
        int SVstart = inData.indexOf('V');
        int POSstart = inData.indexOf('P');
        int servoNum = inData.substring(SVstart + 1, POSstart).toInt();
        int servoPOS = inData.substring(POSstart + 1).toInt();
        if (servoNum == 0)
        {
          servo0.write(servoPOS);
        }
//        if (servoNum == 1)
//        {
//          servo1.write(servoPOS);
//        }
//        if (servoNum == 2)
//        {
//          servo2.write(servoPOS);
//        }
//        if (servoNum == 3)
//        {
//          servo3.write(servoPOS);
//        }
//        if (servoNum == 4)
//        {
//          servo4.write(servoPOS);
//        }
//        if (servoNum == 5)
//        {
//          servo5.write(servoPOS);
//        }
//        if (servoNum == 6)
//        {
//          servo6.write(servoPOS);
//        }
//        if (servoNum == 7)
//        {
//          servo7.write(servoPOS);
//        }
        Serial.print("Servo Done");
      }


      //-----COMMAND IF INPUT THEN JUMP---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "JF")
      {
        int IJstart = inData.indexOf('X');
        int IJTabstart = inData.indexOf('T');
        int IJInputNum = inData.substring(IJstart + 1, IJTabstart).toInt();
        if (digitalRead(IJInputNum) == HIGH)
        {
          Serial.println("T");
        }
        if (digitalRead(IJInputNum) == LOW)
        {
          Serial.println("F");
        }
      }
      //-----COMMAND SET OUTPUT ON---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "ON")
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, isRelayPin(outputNum) ? LOW : HIGH);  // v2: relay active-LOW
        Serial.print("Done");
      }
      //-----COMMAND SET OUTPUT OFF---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "OF")
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, isRelayPin(outputNum) ? HIGH : LOW);  // v2: relay active-LOW
        Serial.print("Done");
      }
      //-----COMMAND TO WAIT INPUT ON---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "WI")
      {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();
        while (digitalRead(InputNum) == LOW) {
          delay(100);
        }
        Serial.print("Done");
      }
      //-----COMMAND TO WAIT INPUT OFF---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "WO")
      {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();

        //String InputStr =  String("Input" + InputNum);
        //uint8_t Input = atoi(InputStr.c_str ());
        while (digitalRead(InputNum) == HIGH) {
          delay(100);
        }
        Serial.print("Done");
      }
      //-----COMMAND ECHO TEST MESSAGE---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "TM")
      {
        String echo = inData.substring(2);
        Serial.println(echo);
      }




      else
      {
        inData = ""; // Clear recieved buffer
      }
    }
  }
}
