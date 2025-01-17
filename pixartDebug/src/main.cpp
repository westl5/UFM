/*
EE40I6 Capstone Project - UFM - Group 10 
members: westl5, gillg62, xur76, yum77

This code is based on code from Bart Trzynadlowski at the following link:
https://github.com/trzy/PixArt

We are using this as a test to interface between the PixArt PAJ7025R3 Sensor and the ESP-WROOM-32D board. 

Extra commenting has also been added to explain register usage

Our PCB is a different setup than trzy's. It does not contain capacitors (these are still in the circuit but not on the pcb),
and has the following connections (these may be reduced in the future):

  +-------------------+
  |     PAJ7025R3     | 
  |                   |
  |      ooooooo      |
  |                   |
  |                   |
  |   -------------   |
  | o o o o o o o o o |
  +-|-|-|-|-|-|-|-|-|-+
    | | | | | | | | |
    | | | | | | | | (OG) G13/LED_SIDE 
    | | | | | | | (BK) GND 
    | | | | | | (GN) G12/MOSI 
    | | | | | (YE) G11/MISO 
    | | | | (WH) G10/SCK 
    | | | (BL) G9/CSB
    | | (BN) G8/VSYNCH
    | (VT) G6/FOD_TRIGGER
    (RD) VDDMA/VCC

* Connections to ESP-WROOM-32D board headers:
 
  VDDMA          -> 3v3   (recommend pin nearest GND)
  G6/FOD_TRIGGER -> GPIO 01 
  G8/VSYNCH      -> UNUSED
  G9/CSB         -> GPIO 5
  G10/SCK        -> GPIO 18
  G11/MISO       -> GPIO 19
  G12/MOSI       -> GPIO 23
  GND            -> GND

  IR LEDs: OSRAM SF 4356, 860nm wavelength, which is within the PixArt PAJ7025R3's detection range of 800-900nm 
*/


/* ORIGINAL TRZY COMMENTING
 * Reads object data from PixArt sensor using Adafruit nRF52832 Feather.
 * PCB board connector J1 leads:
 *
 *  +--------------+
 *  |    PixArt    |
 *  |              |
 *  | C2        C1 |
 *  |              |
 *  | J1           |
 *  | -----------  |
 *  +-| | | | | |--+
 *    | | | | | |
 *    | | | | | |
 *    | | | | | |
 *    | | | | | GND
 *    | | | | G12/MOSI
 *    | | | |
 *    | | | G11/MISO
 *    | | |
 *    | | G10/SCK
 *    | |
 *    | G9/CSB
 *    |
 *    VDDMA
 *
 * Connections to nRF52832 board headers:
 *
 *    VDDMA    -> +3.3v (recommend pin nearest GND)
 *    G9/CSB   -> SS
 *    G10/SCK  -> SCK
 *    G11/MISO -> MISO (note MISO and MOSI ordering reversed from PixArt PCB)
 *    G12/MOSI -> MOSI
 *    GND      -> GND
 *
 * Vishay Semiconductors TSHA4400 IR LED (875nm) successfully detectable
 * by the PixArt sensor.
 */

#include <cstdio>
#include <cstring>
#include <Arduino.h>
#include <SPI.h> 

#define SPI_CLK_SPEED 14000000 //14MHz is max value for PixArt sensor
#define DEBUG_IMAGE 0x00
  /*
  DSP Settings; test images
  write one of following image_num values to 0x2b register in bank 0x01 to set test image

  0x00: test image off
  0x05: 16 fixed objects
  0x06: 4 fixed objects
  0x07: circling moving objects
  0x0b: 4 fixed 1 pixel boundary objects
  */

static unsigned s_frame_period_micros;

struct PA_object
{
  uint16_t area; 
  uint16_t cx;
  uint16_t cy;
  uint8_t average_brightness;
  uint8_t max_brightness;
  uint8_t range;
  uint8_t radius;
  uint8_t boundary_left;
  uint8_t boundary_right;
  uint8_t boundary_up;
  uint8_t boundary_down;
  uint8_t aspect_ratio;
  uint8_t vx;
  uint8_t vy;
  
  //renders boundary information into a symbol based visualization of the object
  void render(char *output, int pitch, char symbol)
  {
    int lx = min(97, (int) boundary_left);
    int rx = min(97, (int) boundary_right);
    int uy = min(97, (int) boundary_up);
    int dy = min(97, (int) boundary_down);
    Serial.print(lx,DEC);Serial.print(",");Serial.print(rx,DEC);Serial.print(",");Serial.print(uy,DEC);Serial.print(",");Serial.print(dy,DEC);Serial.print("\n");
    for (int y = uy; y <= dy; y++)
    {
      for (int x = lx; x <= rx; x++)
      {
        output[y * pitch + x] = symbol;
      }
    }
  }
  //prints relevant object information over serial
  void print()
  {
    char buffer[1024];
    char *ptr = buffer;
    ptr += sprintf(ptr, "center          = (%d,%d)\n", cx, cy);
    ptr += sprintf(ptr, "area            = %d\n", area);
    ptr += sprintf(ptr, "avg. brightness = %d\n", average_brightness);
    ptr += sprintf(ptr, "max brightness  = %d\n", max_brightness);
    ptr += sprintf(ptr, "range           = %d\n", range);
    ptr += sprintf(ptr, "radius          = %d\n", radius);
    ptr += sprintf(ptr, "boundary        = (%d,%d,%d,%d)\n", boundary_left, boundary_right, boundary_up, boundary_down);
    ptr += sprintf(ptr, "aspect          = %d\n", aspect_ratio);
    ptr += sprintf(ptr, "vx              = %d\n", vx);
    ptr += sprintf(ptr, "vy              = %d\n", vy);
    Serial.print(buffer);
  }
  //loads object data from sensor into PA_object struct
  void load(const uint8_t *data, int format)
  {
    memset(this, 0, sizeof(this));

    // Formats 1-4
    area = data[0] | ((data[1] & 0x3f) << 8);
    cx = data[2] | ((data[3] & 0x0f) << 8);
    cy = data[4] | ((data[5] & 0x0f) << 8);

    // Format 1, 3
    if (format == 1 || format == 3)
    {
      average_brightness = data[6];
      max_brightness = data[7];
      range = data[8] >> 4;
      radius = data[8] & 0xf;
    }

    if (format == 1 || format == 4)
    {
      int offset = format == 4 ? 3 : 0;
      boundary_left = data[9 - offset] & 0x7f;
      boundary_right = data[10 - offset] & 0x7f;
      boundary_up = data[11 - offset] & 0x7f;
      boundary_down = data[12 - offset] & 0x7f;
      aspect_ratio = data[13 - offset];
      vx = data[14 - offset];
      vy = data[15 - offset];
    }
  }

  PA_object(const uint8_t *data, int format)
  {
    load(data, format);
  }

  PA_object()
  {
  }
};

//Writes data to specified register using SPI
void PA_write(uint8_t reg, uint8_t data)
{
  SPI.transfer(0x00);     // bit 7 = write (0), bits 6-0 = single byte (0)
  SPI.transfer(reg);
  SPI.transfer(data);
}

//Reads data from specified register using SPI
uint8_t PA_read(uint8_t reg)
{
  SPI.transfer(0x80);     // bit 7 = read (1), bits 6-0 = single byte (0)
  SPI.transfer(reg);
  return SPI.transfer(0);
}

//Burst reads data from specified register using SPI, and stores it in buffer
void PA_burst_read(uint8_t reg_base, uint8_t buffer[], uint16_t num_bytes)
{
  SPI.transfer(0x81); //0b 1000 0001 bit 7 = read (1), bits 6-0 = multyple bytes (1)
  SPI.transfer(reg_base); //specify register to start reading from
  for (uint16_t i = 0; i < num_bytes; i++) // read num_bytes of data
  {
    buffer[i] = SPI.transfer(0);
  }
}

double PA_get_frame_period_microseconds()
{
  PA_write(0xef, 0x0c); // bank C
  uint32_t cmd_frame_period = PA_read(0x07);
  cmd_frame_period |= PA_read(0x08) << 8;
  cmd_frame_period |= PA_read(0x09) << 16;
  double frame_period_100ns = (double) cmd_frame_period;
  double frame_period_millis = frame_period_100ns * 1e-1; // 100 ns -> us
  return frame_period_millis;
}

double PA_get_frame_period_milliseconds()
{
  return 1e-3 * PA_get_frame_period_microseconds();
}

//Prints settings of the sensor
void PA_print_settings()
{
    // Register bank 0
    PA_write(0xef, 0);
    uint16_t product_id = (uint16_t(PA_read(0x03)) << 8) | PA_read(0x02);
    uint16_t dsp_area_max_threshold = (PA_read(0x0c) << 8) | PA_read(0x0b);
    uint8_t dsp_noise_threshold = PA_read(0x0f);
    uint8_t dsp_orientation_ratio = PA_read(0x10);
    uint8_t dsp_orientation_factor = PA_read(0x11);
    uint8_t dsp_max_object_num = PA_read(0x19);

    // Register bank 1
    PA_write(0xef, 1);
    uint8_t sensor_gain_1 = PA_read(0x05);
    uint8_t sensor_gain_2 = PA_read(0x06);
    uint16_t sensor_exposure_length = (PA_read(0x0f) << 8) | PA_read(0x0e);

    // Register bank C
    PA_write(0xef, 0x0c);
    uint16_t cmd_scale_resolution_x = (PA_read(0x61) << 8) | PA_read(0x60);
    uint16_t cmd_scale_resolution_y = (PA_read(0x63) << 8) | PA_read(0x62);

    // Reset back to bank 0
    PA_write(0xef, 0);

    // Print everything
    char buffer[2048];
    char *ptr = buffer;
    ptr += sprintf(ptr, "PAJ7025R3 Settings\n");
    ptr += sprintf(ptr, "------------------\n");
    ptr += sprintf(ptr, "Product ID                    = %04x\n", product_id);
    ptr += sprintf(ptr, "DSP area max threshold        = %04x\n", dsp_area_max_threshold);
    ptr += sprintf(ptr, "DSP noise threshold           = %02x\n", dsp_noise_threshold);
    ptr += sprintf(ptr, "DSP orientation ratio         = %02x\n", dsp_orientation_ratio);
    ptr += sprintf(ptr, "DSP orientation factor        = %02x\n", dsp_orientation_factor);
    ptr += sprintf(ptr, "DSP maximum number of objects = %02x\n", dsp_max_object_num);
    ptr += sprintf(ptr, "Sensor gain 1                 = %02x\n", sensor_gain_1);
    ptr += sprintf(ptr, "Sensor gain 2                 = %02x\n", sensor_gain_2);
    ptr += sprintf(ptr, "Sensor exposure length        = %04x\n", sensor_exposure_length);
    ptr += sprintf(ptr, "Scale resolution X            = %04x\n", cmd_scale_resolution_x);
    ptr += sprintf(ptr, "Scale resolution Y            = %04x\n", cmd_scale_resolution_y);
    ptr += sprintf(ptr, "Frame period                  = %1.4f ms\n", PA_get_frame_period_milliseconds());
    Serial.print(buffer);
}
//Performs initial settings sequence as per PAJ7025R3 datasheet
void PA_load_initial_settings()
{
  //0xef is the bank select register, holds value bank_no[7:0]
  //switch to bank 0x00
  PA_write(0xef, 0x00); 
  PA_write(0xdc, 0x00); //as per initial settings sequence, register purpose not specified in datasheet
  PA_write(0xfb, 0x04); //as per initial settings sequence, register purpose not specified in datasheet

  //switch to bank 0x00
  PA_write(0xef, 0x00); 
  PA_write(0x2f, 0x05); //set cmd_manual_powercontrol_sensor_on to 1 at bits 0 and 2 (power on sensor)
  PA_write(0x30, 0x00); //make manual power control effective by first setting update flag 0
  PA_write(0x30, 0x01); //then set flag to 1
  PA_write(0x1f, 0x00); //as per initial settings sequence, register purpose not specified in datasheet
  
  //switch to bank 0x01
  PA_write(0xef, 0x01); 
  PA_write(0x2d, 0x00); 

  //switch to bank 0x0C
  PA_write(0xef, 0x0c); 
  PA_write(0x64, 0x00); //purpose of registers 0x12,0x13 and 0x64-0x72 not specified in datasheet
  PA_write(0x65, 0x00);
  PA_write(0x66, 0x00);
  PA_write(0x67, 0x00);
  PA_write(0x68, 0x00);
  PA_write(0x69, 0x00);
  PA_write(0x6a, 0x00);
  PA_write(0x6b, 0x00);
  PA_write(0x6c, 0x00);
  PA_write(0x71, 0x00);
  PA_write(0x72, 0x00);
  PA_write(0x12, 0x00);
  PA_write(0x13, 0x00); 

  PA_write(0xef, 0x00);  //switch to bank 0x00
  PA_write(0x01, 0x01);  // APPLY_COMMAND_2 (bank0_synch_updated_flag = 1)
}
//Set frame rate for settings for the sensor, only applicable in STREAM mode
void PA_set_frame_rate(double hz)
{
  //Default frame rate is 200.88 Hz
  //Default cmd_frame_period is 0x00C274 = 49780, 100ns*49780 = 4.978 ms
  double period = 1.0 / hz;
  double period_100ns = 1e7 * period; //convert period to 100ns units
  uint32_t cmd_frame_period = (uint32_t) round(period_100ns);

  //cmd_frame period is a 20-bit value [19:0] from registers 0x07, 0x08, 0x09 in bank 0x0C
  PA_write(0xef, 0x0c); // switch to bank 0x0C
  PA_write(0x07, cmd_frame_period & 0xff); //cmd_frame_period [7:0] stored in 0x07
  PA_write(0x08, (cmd_frame_period >> 8) & 0xff); //cmd_frame_period [15:8] stored in 0x08
  PA_write(0x09, (cmd_frame_period >> 16) & 0xf); //cmd_frame_period [19:16] stored in 0x09

  char buffer[2048];
  sprintf(buffer, "cmd_frame_period = 0x%05x\n", cmd_frame_period);
  Serial.print(buffer); 
}

void PA_set_sensor_exposure_time(double time)
{
  //Minimum allowable exposure time = 20us 
  //Maximum allowable exposure time = frame_period - 2.7ms
  double exposure_time_200ns = time / 200e-9;
  uint32_t b_expo = (unsigned int) round(exposure_time_200ns);

  PA_write(0xef, 0x0c); //switch to bank 0x0C
  //b_expo is a 16 bit value [15:0] stored in registers 0x0F, 0x10
  PA_write(0x0f, b_expo & 0xff); //b_expo [7:0] stored in 0x0F
  PA_write(0x10, (b_expo >> 8) & 0xff); //b_expo [15:8] stored in 0x10
  PA_write(0xef, 0x01); // APPLY_COMMAND_1 (switch to reigster bank 0x01)
  PA_write(0x01, 0x01); // APPLY_COMMAND_2 (bank1_synch_updated_flag)

  //NOTE: new exposure settings will take effect 2 frames after setting the update flag 

  char buffer[2048];
  sprintf(buffer, "b_expo = 0x%04x\n", b_expo);
  Serial.print(buffer);
}

void PA_set_sensor_gain(uint8_t b_global, uint8_t b_ggh)
{
  /*
  gain = (1 + b_global/16) * H

  | b_ggh | 0 | 2 | 3 |
  | H     | 1 | 2 | 4 |

  Sensor Gain Table (from datasheet)
  +------------------------------------------+
  |     Gain      |          b_ggh           |
  |               |  0x00  |  0x02  |  0x03  |
  +------------------------------------------+
  |          0x00 | 1.0000 | 2.0000 | 4.0000 |
  |          0x01 | 1.0625 | 2.1250 | 4.2500 |
  |          0x02 | 1.1250 | 2.2500 | 4.5000 |
  |          0x03 | 1.1875 | 2.3750 | 4.7500 |
  |          0x04 | 1.2500 | 2.5000 | 5.0000 |
  |          0x05 | 1.3125 | 2.6250 | 5.2500 |
  |          0x06 | 1.3750 | 2.7500 | 5.5000 |
  |          0x07 | 1.4375 | 2.8750 | 5.7500 |
  | b_global 0x08 | 1.5000 | 3.0000 | 6.0000 |
  |          0x09 | 1.5625 | 3.1250 | 6.2500 |
  |          0x0a | 1.6250 | 3.2500 | 6.5000 |
  |          0x0b | 1.6875 | 3.3750 | 6.7500 |
  |          0x0c | 1.7500 | 3.5000 | 7.0000 |
  |          0x0d | 1.8125 | 3.6250 | 7.2500 |
  |          0x0e | 1.8750 | 3.7500 | 7.5000 |
  |          0x0f | 1.9375 | 3.8750 | 7.7500 |
  |          0x10 | 2.0000 | 4.0000 | 8.0000 |
  +------------------------------------------+
  */
 
  PA_write(0xef, 0x0c); // switch to bank 0x0C
  PA_write(0x0b, b_global & 0x1f); // b_global [4:0] stored in 0x0B
  PA_write(0x0c, b_ggh & 3);  // b_ggh [1:0] stored in 0x0C
  PA_write(0xef, 0x01); // APPLY_COMMAND_1 (switch to register bank 0x01)
  PA_write(0x01, 0x01); // APPLY_COMMAND_2 (bank1_synch_updated_flag)

  //NOTE: new exposure settings will take effect 2 frames after setting the update flag 
}

void PA_set_debug_image(uint8_t image_num)
{
  /*
  DSP Settings; test images
  write one of following image_num values to 0x2b register in bank 0x01 to set test image

  0x00: test image off
  0x05: 16 fixed objects
  0x06: 4 fixed objects
  0x07: circling moving objects
  0x0b: 4 fixed 1 pixel boundary objects
  */

  PA_write(0xef, 0x01); // bank 1
  PA_write(0x2b, image_num);
  PA_write(0xef, 0x01); // APPLY_COMMAND_1 (needed?)
  PA_write(0x01, 0x01); // APPLY_COMMAND_2 (...)
}

void PA_read_report(PA_object objs[16], int format)
{
  uint8_t report[256];
  int num_bytes = 256;
  uint8_t format_code = 5;

  switch (format)
  {
    default:
      break;
    case 1: // format 1: 256-byte, register bank 0x05, Address 0x00-0xFF
      num_bytes = 256;
      format_code = 5;
      break;
    case 2: // format 2: 96-byte, register bank 0x09, Address 0x00-0x5F
      num_bytes = 96;
      format_code = 9;
      break;
    case 3: // format 3: 144-byte, register bank 0x0A, Address 0x00-0x8F
      num_bytes = 144;
      format_code = 10;
      break;
    case 4: // format 4: 208-byte, register bank 0x0B, Address 0x00-0xCF
      num_bytes = 208;
      format_code = 11;
      break;
  }

  PA_write(0xef, format_code); //switch to register bank with desired data format
  PA_burst_read(0, report, num_bytes); //read all sensor data into report buffer to get object data

  int label_size = num_bytes / 16;
  for (int i = 0; i < 16; i++)
  {
    objs[i].load(&report[i * label_size], format);
  }
}

void PA_init()
{

  // SPI begin
  SPI.beginTransaction(SPISettings(SPI_CLK_SPEED, LSBFIRST, SPI_MODE3));

  // Set up
  digitalWrite(SS, 0);  // chip select pulled low to start communication with sensor
  PA_load_initial_settings(); //load initial settings as per datasheet sequence

  //PA_set_frame_rate(30);
  //PA_set_frame_rate(200.88);
  //PA_set_sensor_exposure_time(1.6384e-3);
  //PA_set_sensor_gain(0x10, 0);

  PA_set_debug_image(DEBUG_IMAGE); //turn off debug image mode 
  PA_print_settings(); //print settings to serial monitor
  s_frame_period_micros = (unsigned) PA_get_frame_period_microseconds(); //get frame period in microseconds

  digitalWrite(SS, 1); //deassert chip select to end SPI transaction

  // Read first frame from sensor
  for (int i = 0; i < 1; i++)
  {
    delayMicroseconds(s_frame_period_micros);
  }
  digitalWrite(SS, 0); //CSB low to start SPI transaction
  PA_object objs[16]; //create object array to store object data
  PA_read_report(objs, 1); //read object data into object array
  digitalWrite(SS, 1); //CSB high to end SPI transaction

  // SPI end
  //SPI.endTransaction();

  // Print detected objects from the first frame read
  for (int i = 0; i < 16; i++)
  {
    Serial.print("Object ");
    Serial.print(i, DEC);
    Serial.print("\n");
    Serial.print("--------");
    Serial.print(i >= 10 ? "-\n" : "\n");
    objs[i].print();
  }

  // Render image of detected objects in first frame
  char *image = (char *) malloc((98 + 1) * 98 + 1);
  memset(image, '.', 99 * 98);
  for (int y = 0; y < 98; y++)
  {
    image[y * 99 + 98] = '\n';
  }
  image[99 * 98] = 0;

  for (int i = 0; i < 16; i++)
  {
    char symbol = i < 10 ? ('0' + i) : ('a' + i - 10);
    objs[i].render(image, 99, symbol);
  }
  Serial.print("Image:\n");
  Serial.print(image);
  free(image);
}

void setup() {

  Serial.begin(9600); //serial communication using UART through USB-C to get data from sensor (will be chanaged to bluetooth later)

  pinMode(SS, OUTPUT); //chip select pin
  digitalWrite(SS, 1); //deassert chip select to start
  SPI.begin(); //init SPI communication

  PA_init(); //initialize sensor


  //SPI.end();

  //digitalWrite(SS, 1);
  //SPI.beginTransaction(SPISettings(14000000, LSBFIRST, SPI_MODE3));
}

void loop() {

  // put your main code here, to run repeatedly
  //Serial.print("Loop Start\n");
  delayMicroseconds(s_frame_period_micros);
  digitalWrite(SS, 0); //Assert CSB Low
  //PA_print_settings();
  PA_object objs[16]; //initialize objects array
  PA_read_report(objs, 1);
  digitalWrite(SS, 1);  // deasserting CS seems to be required for next frame readout
  
  char buffer[1024];
  char *ptr = buffer;
  ptr += sprintf(ptr, "%d, %d, %d, %d, %d, %d, %d, %d\n", objs[0].cx,objs[0].cy, objs[1].cx,objs[1].cy, objs[2].cx,objs[2].cy, objs[3].cx, objs[3].cy);

  //int numObjects  = 3;
  /*
  for (int i = 0; i < numObjects; i++)
  {
    ptr += sprintf(ptr, "Obj%d, %d, %d, %d\n", i, objs[i].cx, objs[i].cy, objs[i].area);
  }
  */
  Serial.print(buffer);
  
  delay(17);
 
}
