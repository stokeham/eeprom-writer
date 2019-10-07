// EEPROM Programmer - code for an Arduino (moved to shift register for address bits)
//
// Written by K Adcock. Edit by CJC for UNO
//       Jan 2016 - Initial release
//       Dec 2017 - Slide code tartups, to remove compiler errors for new Arduino IDE (1.8.5).
//   7th Dec 2017 - Updates from Dave Curran of Tynemouth Software, adding commands to enable/disable SDP.
//  10th Dec 2017 - Fixed one-byte EEPROM corruption (always byte 0) when unprotecting an EEPROM
//                  (doesn't matter if you write a ROM immediately after, but does matter if you use -unprotect in isolation)
//                - refactored code a bit (split loop() into different functions)
//                - properly looked at timings on the Atmel datasheet, and worked out that my delays
//                  during reads and writes were about 10,000 times too big!
//                  Reading and writing is now orders-of-magnitude quicker.
//  
// Distributed under an acknowledgement licence, because I'm a shallow, attention-seeking tart. :)
//
// http://danceswithferrets.org/geekblog/?page_id=903
//
// This software presents a 9600-8N1 serial port.
//
// R[hex address]                         - reads 16 bytes of data from the EEPROM
// W[hex address]:[data in two-char hex]  - writes up to 16 bytes of data to the EEPROM
// P                                      - set write-protection bit (Atmels only, AFAIK)
// U                                      - clear write-protection bit (ditto)
// V                                      - prints the version string
//
// Any data read from the EEPROM will have a CRC checksum appended to it (separated by a comma).
// If a string of data is sent with an optional checksum, then this will be checked
// before anything is written.
//

#include <avr/pgmspace.h>

const char hex[] =
{
  '0', '1', '2', '3', '4', '5', '6', '7',
  '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};

const char version_string[] = {"EEPROM Version=0.03"};

// pins for Uno version
static const int SHIFT_DATA   = 2;
static const int SHIFT_CLK    = 3;
static const int SHIFT_LATCH  = 4;
static const int kPin_Data0   = 5;
static const int kPin_Data1   = 6;
static const int kPin_Data2   = 7;
static const int kPin_Data3   = 8;
static const int kPin_Data4   = 9;
static const int kPin_Data5   = 10;
static const int kPin_Data6   = 11;
static const int kPin_Data7   = 12;
static const int kPin_nWE     = 13;

byte g_cmd[80]; // strings received from the controller will go in here
static const int kMaxBufferSize = 16;
byte buffer[kMaxBufferSize];

static const long int k_uTime_WritePulse_uS = 1; 
static const long int k_uTime_ReadPulse_uS = 1;
// (to be honest, both of the above are about ten times too big - but the Arduino won't reliably
// delay down at the nanosecond level, so this is the best we can do.)

// the setup function runs once when you press reset or power the board
void setup()
{
  Serial.begin(9600);

  // shift register addressing pins as outputs
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);


  // control lines are ALWAYS outputs
  pinMode(kPin_nWE, OUTPUT); digitalWrite(kPin_nWE, HIGH); // not writing

  SetDataLinesAsInputs();
  SetAddress(0, false); //intial address to 0 with OE high(off)
}

void loop()
{
  while (true)
  {
    ReadString();
    
    switch (g_cmd[0])
    {
      case 'V': Serial.println(version_string); break;
      case 'P': SetSDPState(true); break;
      case 'U': SetSDPState(false); break;
      case 'R': ReadEEPROM(); break;
      case 'W': WriteEEPROM(); break;
      case 0: break; // empty string. Don't mind ignoring this.
      default: Serial.println("ERR Unrecognised command"); break;
    }
  }
}

void ReadEEPROM() // R<address>  - read kMaxBufferSize bytes from EEPROM, beginning at <address> (in hex)
{
  if (g_cmd[1] == 0)
  {
    Serial.println("ERR");
    return;
  }

  // decode ASCII representation of address (in hex) into an actual value
  int addr = 0;
  int x = 1;
  while (x < 5 && g_cmd[x] != 0)
  {
    addr = addr << 4;
    addr |= HexToVal(g_cmd[x++]);
  }     

  digitalWrite(kPin_nWE, HIGH); // disables write
  SetDataLinesAsInputs();
  delayMicroseconds(1);
      
  ReadEEPROMIntoBuffer(addr, kMaxBufferSize);

  // now print the results, starting with the address as hex ...
  Serial.print(hex[ (addr & 0xF000) >> 12 ]);
  Serial.print(hex[ (addr & 0x0F00) >> 8  ]);
  Serial.print(hex[ (addr & 0x00F0) >> 4  ]);
  Serial.print(hex[ (addr & 0x000F)       ]);
  Serial.print(":");
  PrintBuffer(kMaxBufferSize);

  Serial.println("OK");
  
//  digitalWrite(kPin_nOE, HIGH); // stops the EEPROM outputting the byte
}

void WriteEEPROM() // W<four byte hex address>:<data in hex, two characters per byte, max of 16 bytes per line>
{
  if (g_cmd[1] == 0)
  {
    Serial.println("ERR");
    return;
  }

  int addr = 0;
  int x = 1;
  while (g_cmd[x] != ':' && g_cmd[x] != 0)
  {
    addr = addr << 4;
    addr |= HexToVal(g_cmd[x]);
    ++x;
  }

  // g_cmd[x] should now be a :
  if (g_cmd[x] != ':')
  {
    Serial.println("ERR");
    return;
  }
  
  x++; // now points to beginning of data
  uint8_t iBufferUsed = 0;
  while (g_cmd[x] && g_cmd[x+1] && iBufferUsed < kMaxBufferSize && g_cmd[x] != ',')
  {
    uint8_t c = (HexToVal(g_cmd[x]) << 4) | HexToVal(g_cmd[x+1]);
    buffer[iBufferUsed++] = c;
    x += 2;
  }

  // if we're pointing to a comma, then the optional checksum has been provided!
  if (g_cmd[x] == ',' && g_cmd[x+1] && g_cmd[x+2])
  {
    byte checksum = (HexToVal(g_cmd[x+1]) << 4) | HexToVal(g_cmd[x+2]);

    byte our_checksum = CalcBufferChecksum(iBufferUsed);

    if (our_checksum != checksum)
    {
      // checksum fail!
      iBufferUsed = -1;
      Serial.print("ERR ");
      Serial.print(checksum, HEX);
      Serial.print(" ");
      Serial.print(our_checksum, HEX);
      Serial.println("");
      return;
    }
  }

  // buffer should now contains some data
  if (iBufferUsed > 0)
  {
    WriteBufferToEEPROM(addr, iBufferUsed);
  }

  if (iBufferUsed > -1)
  {
    Serial.println("OK");
  }
}

// Important note: the EEPROM needs to have data written to it immediately after sending the "unprotect" command, so that the buffer is flushed.
// So we read byte 0 from the EEPROM first, then use that as the dummy write afterwards.
// It wouldn't matter if this facility was used immediately before writing an EEPROM anyway ... but it DOES matter if you use this option
// in isolation (unprotecting the EEPROM but not changing it).

void SetSDPState(bool bWriteProtect)
{

  digitalWrite(kPin_nWE, HIGH); // disables write
  SetDataLinesAsInputs();
  
  byte bytezero = ReadByteFrom(0);
  
  SetDataLinesAsOutputs();

  if (bWriteProtect)
  {
    WriteByteTo(0x1555, 0xAA);
    WriteByteTo(0x0AAA, 0x55);
    WriteByteTo(0x1555, 0xA0);
  }
  else
  {
    WriteByteTo(0x1555, 0xAA);
    WriteByteTo(0x0AAA, 0x55);
    WriteByteTo(0x1555, 0x80);
    WriteByteTo(0x1555, 0xAA);
    WriteByteTo(0x0AAA, 0x55);
    WriteByteTo(0x1555, 0x20);
  }
  
  WriteByteTo(0x0000, bytezero); // this "dummy" write is required so that the EEPROM will flush its buffer of commands.

  Serial.print("OK SDP ");
  if (bWriteProtect)
  {
    Serial.println("enabled");
  }
  else
  {
    Serial.println("disabled");
  }
}

// ----------------------------------------------------------------------------------------

void ReadEEPROMIntoBuffer(int addr, int size)
{
  digitalWrite(kPin_nWE, HIGH);
  SetDataLinesAsInputs();
  
  for (int x = 0; x < size; ++x)
  {
    buffer[x] = ReadByteFrom(addr + x);
  }
}

void WriteBufferToEEPROM(int addr, int size)
{
  digitalWrite(kPin_nWE, HIGH); // disables write
  SetDataLinesAsOutputs();

  for (uint8_t x = 0; x < size; ++x)
  {
    WriteByteTo(addr + x, buffer[x]);
  }
  
}

// ----------------------------------------------------------------------------------------

// this function assumes that data lines have already been set as INPUTS, and that
// nOE is set LOW.
byte ReadByteFrom(int addr)
{
  SetAddress(addr, true);
  delayMicroseconds(k_uTime_ReadPulse_uS);
  byte b = ReadData();

  return b;
}

// this function assumes that data lines have already been set as OUTPUTS, and that
// nOE is set HIGH.
void WriteByteTo(int addr, byte b)
{
  SetAddress(addr, false);
  SetData(b);

  digitalWrite(kPin_nWE, LOW); // enable write  
  delayMicroseconds(k_uTime_WritePulse_uS);
  
  digitalWrite(kPin_nWE, HIGH); // disable write
}

// ----------------------------------------------------------------------------------------

void SetDataLinesAsInputs()
{
  pinMode(kPin_Data0, INPUT);
  pinMode(kPin_Data1, INPUT);
  pinMode(kPin_Data2, INPUT);
  pinMode(kPin_Data3, INPUT);
  pinMode(kPin_Data4, INPUT);
  pinMode(kPin_Data5, INPUT);
  pinMode(kPin_Data6, INPUT);
  pinMode(kPin_Data7, INPUT);
}

void SetDataLinesAsOutputs()
{
  pinMode(kPin_Data0, OUTPUT);
  pinMode(kPin_Data1, OUTPUT);
  pinMode(kPin_Data2, OUTPUT);
  pinMode(kPin_Data3, OUTPUT);
  pinMode(kPin_Data4, OUTPUT);
  pinMode(kPin_Data5, OUTPUT);
  pinMode(kPin_Data6, OUTPUT);
  pinMode(kPin_Data7, OUTPUT);
}

/*
 * Output the address bits and outputEnable signal using shift registers.
 */
void SetAddress(int address, bool outputEnable) {
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, (address >> 8) | (outputEnable ? 0x00 : 0x80));
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, address);

  digitalWrite(SHIFT_LATCH, LOW);
  digitalWrite(SHIFT_LATCH, HIGH);
  digitalWrite(SHIFT_LATCH, LOW);
}

// this function assumes that data lines have already been set as OUTPUTS.
void SetData(byte b)
{
  digitalWrite(kPin_Data0, (b&1)?HIGH:LOW  );
  digitalWrite(kPin_Data1, (b&2)?HIGH:LOW  );
  digitalWrite(kPin_Data2, (b&4)?HIGH:LOW  );
  digitalWrite(kPin_Data3, (b&8)?HIGH:LOW  );
  digitalWrite(kPin_Data4, (b&16)?HIGH:LOW );
  digitalWrite(kPin_Data5, (b&32)?HIGH:LOW );
  digitalWrite(kPin_Data6, (b&64)?HIGH:LOW );
  digitalWrite(kPin_Data7, (b&128)?HIGH:LOW);
}

// this function assumes that data lines have already been set as INPUTS.
byte ReadData()
{
  byte b = 0;

  if (digitalRead(kPin_Data0) == HIGH) b |= 1;
  if (digitalRead(kPin_Data1) == HIGH) b |= 2;
  if (digitalRead(kPin_Data2) == HIGH) b |= 4;
  if (digitalRead(kPin_Data3) == HIGH) b |= 8;
  if (digitalRead(kPin_Data4) == HIGH) b |= 16;
  if (digitalRead(kPin_Data5) == HIGH) b |= 32;
  if (digitalRead(kPin_Data6) == HIGH) b |= 64;
  if (digitalRead(kPin_Data7) == HIGH) b |= 128;

  return(b);
}

// ----------------------------------------------------------------------------------------

void PrintBuffer(int size)
{
  uint8_t chk = 0;

  for (uint8_t x = 0; x < size; ++x)
  {
    Serial.print(hex[ (buffer[x] & 0xF0) >> 4 ]);
    Serial.print(hex[ (buffer[x] & 0x0F)      ]);

    chk = chk ^ buffer[x];
  }

  Serial.print(",");
  Serial.print(hex[ (chk & 0xF0) >> 4 ]);
  Serial.print(hex[ (chk & 0x0F)      ]);
  Serial.println("");
}

void ReadString()
{
  int i = 0;
  byte c;

  g_cmd[0] = 0;
  do
  {
    if (Serial.available())
    {
      c = Serial.read();
      if (c > 31)
      {
        g_cmd[i++] = c;
        g_cmd[i] = 0;
      }
    }
  } 
  while (c != 10);
}

uint8_t CalcBufferChecksum(uint8_t size)
{
  uint8_t chk = 0;

  for (uint8_t x = 0; x < size; ++x)
  {
    chk = chk ^  buffer[x];
  }

  return(chk);
}

// converts one character of a HEX value into its absolute value (nibble)
byte HexToVal(byte b)
{
  if (b >= '0' && b <= '9') return(b - '0');
  if (b >= 'A' && b <= 'F') return((b - 'A') + 10);
  if (b >= 'a' && b <= 'f') return((b - 'a') + 10);
  return(0);
}
