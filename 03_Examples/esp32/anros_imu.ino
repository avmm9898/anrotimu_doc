/* Use Uart Pin 25,27
   Tested on Wemos lolin32
*/
#define RXD2 25
#define TXD2 27
#include "imu_data_decode.h"
#include "packet.h"

uint32_t old_frame_ctr = 0;

void setup() {
  //serial for printing out to screen
  Serial.begin(115200);
    
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  imu_data_decode_init();
}

void loop() {
  while (Serial2.available()) {
    char c = Serial2.read();
    packet_decode(c);
  }

  //if new frame is received, print out
  if (frame_count > old_frame_ctr) {
    old_frame_ctr = frame_count;

    //Hi221/226/229, ch100/110
    if (receive_gwsol.tag != KItemGWSOL) {
      Serial.println(String(receive_imusol.id) + ":"
                     + String(receive_imusol.eul[0]) + ","
                     + String(receive_imusol.eul[1]) + ","
                     + String(receive_imusol.eul[2])
                    );
    }
    //Hi221 dongle
    else {
      for (int i = 0; i < receive_gwsol.n; i++)
      {
        //show timestamp(ms) from startup
        uint32_t ts = esp_timer_get_time() / 1000;
        Serial.println(String(ts) + ":"
                       + String(receive_gwsol.receive_imusol[i].id) + ","
                       + String(receive_gwsol.receive_imusol[i].eul[0]) + ","
                       + String(receive_gwsol.receive_imusol[i].eul[1]) + ","
                       + String(receive_gwsol.receive_imusol[i].eul[2])
                      );

      }
    }
  }
}
