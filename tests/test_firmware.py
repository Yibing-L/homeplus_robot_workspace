"""Compile the sketch as host C++ with fake hardware and exercise its protocol.

This catches parser/control logic errors, not AVR memory use or physical control.
"""
import binascii
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
PRELUDE = r'''
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sstream>
#include <cassert>
#include <algorithm>
using std::max; using std::min; using std::abs; using std::isfinite;
#define PI 3.141592653589793
#define OUTPUT 1
#define HIGH 1
#define LOW 0
#define DIRECT 0
#define AUTOMATIC 1
#define MANUAL 0
#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
unsigned long fakeTime = 1;
unsigned long millis(){return fakeTime;}
void delay(int){} void delayMicroseconds(int){}
void pinMode(int,int){} void digitalWrite(int,int){} void analogWrite(int,int){}
struct FakeSerial {
 std::ostringstream out;
 std::string input;
 void begin(int){} int available(){return input.size();}
 char read(){char c=input[0];input.erase(0,1);return c;}
 template<class T> void print(T x){out<<x;}
 template<class T> void print(T x,int){out<<x;}
 template<class T> void println(T x){out<<x<<"\n";}
 template<class T> void println(T x,int){out<<x<<"\n";}
} Serial;
struct Servo {void attach(int){} void writeMicroseconds(int){}};
struct Encoder {Encoder(int,int){} long read(){return 0;}};
struct AS5600 {void begin(){} int readAngle(){return 2048;}};
struct FakeWire {void begin(){} void beginTransmission(int){} void write(int){} void endTransmission(){}} Wire;
struct PID {
 PID(double*,double*,double*,double,double,double,int){}
 void SetSampleTime(int){} void SetOutputLimits(double,double){} void SetMode(int){}
 void SetTunings(double,double,double){} void Compute(){}
};
'''


def wire(queue, command_id=42):
    return f'CMD {command_id} {binascii.crc_hqx(queue.encode(),0xffff):04x} {queue}'


class FirmwareTests(unittest.TestCase):
    def test_compile_parser_timing_watchdog(self):
        sketch=(ROOT/'ARDUINO_FIRMWARE_UPDATED.txt').read_text()
        sketch=re.sub(r'^#include.*\n','',sketch,flags=re.M)
        prototypes=[]
        for match in re.finditer(r'^(?:void|bool|float|int|int32_t)\s+\w+\([^;{}]*\)\s*\{',sketch,re.M):
            prototypes.append(match.group(0).rstrip().removesuffix('{').strip()+';')
        at=sketch.index('void setup() {')
        sketch=sketch[:at]+'\n'.join(prototypes)+'\n'+sketch[at:]
        good=wire('0 0 0 0 0 0 0 0 0 0 0,100 0 0 0 0 0 0 0 0 0 1000')
        malformed=wire('0 0 0 0 0 0 0 0 0 0 0,100 0 0 0 0 0 0 0 0 0 0')
        bad_limits=wire('0 0 0 0 0 0 150 0 0 0 0')
        # Deliberately keep original checksum after changing a numeric field.
        corrupt=good.replace(',100 ', ',900 ')
        def stream(seq, final, points):
            body=f'{seq} {final} {points}'
            return f'STR 42 {binascii.crc_hqx(body.encode(),0xffff):04x} {body}'
        first=stream(0,0,'0 0 0 0 0 0 0 0 0 0 0,0 0 0 0 0 0 0 0 0 0 1000')
        last=stream(1,1,'0 0 0 0 0 0 0 0 0 0 2000,0 0 0 0 0 0 0 0 0 0 3000')
        main=r'''
int main(){
 setup();
 char good[]="GOOD";
 current_millis=100;lastHostMillis=100;
 elbowOutput=42;shoulderOutput=17;
 parseCommandQueue(good);
 assert(elbowOutput==42 && shoulderOutput==17);
 double heldInput=0,heldOutput=42,heldSetpoint=0;
 float heldAngle=0,heldError=0;
 int heldPWM=1500;
 updateArmJointPID(ELBOW_PID,elbow,elbowPID,heldInput,heldOutput,heldSetpoint,
   armRawToAngle(ELBOW_PID,2048),heldAngle,heldError,heldPWM,.02);
 assert(heldOutput==42);
 assert(commandCount==2 && activeCommandId==42);
 fakeTime=600;lastHostMillis=600;lastArmPIDMillis=600;
 loop();
 assert(fabs(linearCountsToCm(target_x)-50)<0.01);
 dx=0;dy=0;dtheta=0;delbow=0;dshoulder=0;
 fakeTime=2201;lastArmPIDMillis=2201;
 loop();
 assert(commandCount==0);
 assert(Serial.out.str().find("HOST_LOST")!=std::string::npos);
 char bad[]="MALFORMED";parseCommandQueue(bad);assert(commandCount==0);
 assert(Serial.out.str().find("BAD_TIMING")!=std::string::npos);
 char bounds[]="BOUNDS";parseCommandQueue(bounds);assert(commandCount==0);
 assert(Serial.out.str().find("JOINT_LIMIT")!=std::string::npos);
 char corrupt[]="CORRUPT";parseCommandQueue(corrupt);assert(commandCount==0);
 assert(Serial.out.str().find("CRC_MISMATCH")!=std::string::npos);
 // Append while running; preserve time origin and never emit DONE at boundary.
 stopAtCurrentState();x=0;y=0;theta=0;dx=dy=dtheta=delbow=dshoulder=0;
 current_millis=3000;
 char first[]="FIRST";parseStreamPacket(first);
 assert(streamActive && !streamFinal && streamSequence==1);
 unsigned long origin=trajectoryStart;
 current_millis=3500;
 char last[]="LAST";parseStreamPacket(last);
 assert(trajectoryStart==origin && commandCount==4 && streamFinal);
 fakeTime=4100;lastHostMillis=4100;lastArmPIDMillis=4100;
 loop();assert(commandCount==4 && currentCommandIndex==2);
 // Reclaim consumed points on append; no restart of the trajectory clock.
 streamFinal=false;
 char more[]="MORE";parseStreamPacket(more);
 assert(commandCount==4 && currentCommandIndex==1 && trajectoryStart==origin);
 // Final completion happens once, only after final time and measured dwell.
 fakeTime=7100;lastHostMillis=7100;lastArmPIDMillis=7100;dx=dy=dtheta=delbow=dshoulder=0;
 loop();assert(commandCount==4);
 fakeTime=7700;lastHostMillis=7700;lastArmPIDMillis=7700;
 loop();assert(commandCount==0);
 assert(Serial.out.str().find("DONE 42")!=std::string::npos);
 current_millis=8000;
 char again[]="FIRST";parseStreamPacket(again);
 char lastAgain[]="LAST";parseStreamPacket(lastAgain);
 // Duplicate packet rejects the stream rather than repeating movement.
 char duplicate[]="LAST";parseStreamPacket(duplicate);
 assert(commandCount==0);
 assert(Serial.out.str().find("STREAM_SEQUENCE")!=std::string::npos);
 // Running out of non-final data is a fault, never a successful endpoint.
 current_millis=5000;
 char underrun[]="FIRST";parseStreamPacket(underrun);
 fakeTime=6100;lastHostMillis=6100;lastArmPIDMillis=6100;dx=dy=dtheta=delbow=dshoulder=0;
 loop();assert(commandCount==0);
 assert(Serial.out.str().find("BUFFER_UNDERRUN")!=std::string::npos);
 // Receive a real packet in receiver-acknowledged blocks through loop().
 stopAtCurrentState();
 std::string packet=std::string("FIRST")+"\n";
 for(size_t i=0;i<packet.size();i+=32){
   Serial.input=packet.substr(i,32);
   fakeTime=8000;lastHostMillis=8000;lastArmPIDMillis=8000;
   dx=dy=dtheta=delbow=dshoulder=0;
   loop();
   if(i+32<packet.size()){
     assert(Serial.out.str().find("RX4 "+std::to_string(i+32))!=std::string::npos);
   }
 }
 assert(commandCount==2 && streamActive);
 // Coarse transit tolerates the reported arm lag and larger base lag mid-move.
 streamTransit=true;
 target_x=(0.08 / wheel_circumference)*resolution;
 dshoulder=3.21;shoulder_pos=-3.21;
 fakeTime=8000;lastHostMillis=8000;lastArmPIDMillis=8000;
 loop();assert(commandCount==2);
 assert(!status()); // The same error cannot qualify as a reached endpoint.
 streamTransit=false;shoulder_pos=0;dshoulder=0;
 // The reported 4.06 cm lag is allowed; lag beyond 6 cm still aborts.
 target_x = (0.0406 / wheel_circumference) * resolution;
 fakeTime=8001;lastHostMillis=8001;lastArmPIDMillis=8001;
 loop();assert(commandCount==2);
 target_x = (0.061 / wheel_circumference) * resolution;
 fakeTime=8002;lastHostMillis=8002;lastArmPIDMillis=8002;
 loop();assert(commandCount==0);
 assert(Serial.out.str().find("TRACKING_ERROR dx_cm=")!=std::string::npos);
 // Transit acceptance is explicit; precision/contact retains stricter bounds.
 streamActive=true;streamTransit=true;
 dx=0;dy=0;dtheta=0;delbow=.8;dshoulder=.8;dgrip=dhand=dwrist=dla=dframe=0;
 assert(status());streamTransit=false;assert(!status());
 stopAtCurrentState();assert(!streamActive);
 printRobotState();assert(Serial.out.str().find("STATE4 42")!=std::string::npos);
 // Endpoint-only checking still fails if the shoulder never reaches the goal.
 current_millis=12000;
 char endpoint[]="FIRST";parseStreamPacket(endpoint);
 streamTransit=true;streamFinal=true;
 x=y=theta=0;target_x=target_y=target_theta=0;
 shoulder_pos=-3.21;dshoulder=3.21;delbow=0;
 fakeTime=13001;lastHostMillis=13001;lastArmPIDMillis=13001;
 loop();assert(commandCount==2);
 fakeTime=18001;lastHostMillis=18001;lastArmPIDMillis=18001;
 loop();assert(commandCount==0);
 assert(Serial.out.str().find("SETTLE_TIMEOUT")!=std::string::npos);
}
'''.replace('MORE',stream(2,1,'0 0 0 0 0 0 0 0 0 0 4000')).replace('FIRST',first).replace('LAST',last).replace('GOOD',good).replace('MALFORMED',malformed).replace('BOUNDS',bad_limits).replace('CORRUPT',corrupt)
        with tempfile.TemporaryDirectory() as temp:
            source=Path(temp)/'firmware.cpp';source.write_text(PRELUDE+sketch+main)
            binary=Path(temp)/'firmware_test'
            result=subprocess.run(['g++','-std=c++17','-Wall',str(source),'-o',str(binary)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            result=subprocess.run([str(binary)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)


if __name__=='__main__':unittest.main()
