// #include <ens16x.h>

// void print_ens160_compensation_registers() {
//   uint8_t rdata[4];
//   uint16_t temp_raw, hum_raw;
//   float temp_parsed, hum_parsed;

//   ens160.read(ENS16X_REGISTER_ADDRESS_DATA_T, rdata, 4);
//   temp_raw = rdata[1];
//   temp_raw <<= 8;
//   temp_raw |= rdata[0];
//   hum_raw = rdata[3];
//   hum_raw <<= 8;
//   hum_raw |= rdata[2];

//   temp_parsed = ((float) temp_raw / 64) - 273.15;
//   hum_parsed = ((float) hum_raw) / 512;


//   Serial.printf("ENS160 temp comp: raw=%04x, converted=%2.2f\n", temp_raw, temp_parsed);
//   Serial.printf("ENS160 hum  comp: raw=%04x, converted=%2.2f\n", hum_raw, hum_parsed);
// }


// void birb_position_test_loop() {
//   int pos;
//   int degs_per_second;
//   int nargs;
//   const char* rdata;

//   while(1) {
//     if (Serial.available() > 0) {
//       String r = Serial.readStringUntil('\n');
//       rdata = r.c_str();
//       nargs = sscanf(rdata, "pos=%d dps=%d", &pos, &degs_per_second);
//       if (nargs == 1) {
//         set_birb_position(pos, 20);
//       } else if (nargs == 2) {
//         set_birb_position(pos, degs_per_second);
//       } else {
//         Serial.printf("ERROR: Read %d arguments. String='%s'\n", nargs, rdata);
//       }
//     }
//   }
// }