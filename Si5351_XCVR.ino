/*
            Arduino Controlled Si5351A WSPR Tansceiver
This sketch is written for a Si5351A module using a 25 MHz clock frequency and
a Softrock Lite II receiver. The Softrock Lite II's crystal oscillator is replaced
by CLK0 from the Si5351A module. CLK2 of the Si5351A is used as the transmit source. 
Single band opeation is possible from 10 to 630 meters. 

 
 Copyright (C) 2015,  Gene Marcus W3PM GM4YRE
 Permission is granted to use, copy, modify, and distribute this software
 and documentation for non-commercial purposes.


 24 May, 2015


 ------------------------------------------------------------------------
 Uno Digital Pin Allocation

 D0/RX
 D1
 D2 Set LOW on reset to calibrate Si5351A
 D3
 D4
 D5
 D6
 D7  
 D8  
 D9 
 D10 
 D11
 D12 Set LOW to transmit
 D13 Antenna Relay Contol (RX-LOW,TX-HIGH)
 A0/D14
 A1/D15
 A2/D16 
 A3/D17 
 A4/D18 Si5351 SDA
 A5/D19 Si5351 SCL

 ----------------------------------------------------------------
 */

#include "Wire.h"
#include <avr/interrupt.h>


//_________________________Enter home callsign and grid square below:_____________________
char call2[13] = "W3PM";   //e.g. "W3PM" or "GM4YRE"
char locator[7] = "EM64";  // Use 4 character locator e.g. "EM64"
/*
Note:
- Upper or lower case characters are acceptable.
- Compound callsigns may use up to a three letter/number combination prefix followed by a “/”. 
  A one letter or two number suffix may be used preceded by a “/”.
*/


//_________________________Enter power level below:_______________________________________
byte ndbm = 30; // Min = 0 dBm, Max = 43 dBm, steps 0,3,7,10,13,17,20,23,27,30,33,37,40,43


// Receiver dial frequencies in Hz. Last entry must be zero.
const unsigned long RXdialFreq [] = {
  474200,  // Band 0
  1836600, // Band 1
  3592600, // Band 2
  5287200, // Band 3
  7038600, // Band 4
  10138700,// Band 5
  14095600,// Band 6
  18104600,// Band 7
  21094600,// Band 8
  24924600,// Band 9
  28124600,// Band 10
  0
};

// Set default band 
int band = 2; // 80 M

/*
Set 25 MHz Clock Calibration Factor
- Connect frequency counter to the Si5351A CLK1.
- Hold Arduino pin 2 LOW during a reset.
- Annotate counter frequency in Hz.
- Subtract 25 MHz from counter reading. 
- Enter the difference in Hz (i.e. -396) below.
*/
const int CalFactor = -396;

// Transmit offset frequency in Hz. Range = 1400-1600 Hz
int TXoffset = 1525;

// In-band transmit frequency hopping? (1=Yes, 0=No)
const int FreqHopTX = 1;

/*
WSPR I/Q Frequency Offset
- Enter 12000 to enable WSPR I/Q mode
- Enter 0 to disable WSPR I/Q mode
*/
const int IQwspr = 12000;

// Set up MCU pins
#define CalSet             2
#define TX                 12
#define PTT                13
#define Si5351A_addr       0x60
#define CLK0               0
#define CLK1               1
#define CLK2               2
#define CLK_ENABLE_CONTROL 3
#define CLK0_CONTROL       16
#define CLK1_CONTROL       17
#define CLK2_CONTROL       18
#define SYNTH_PLL_A        26
#define SYNTH_PLL_B        34
#define SYNTH_MS_0         42
#define SYNTH_MS_1         50
#define SYNTH_MS_2         58
#define CLK0_PHOFF         165
#define CLK1_PHOFF         166
#define CLK2_PHOFF         167
#define PLL_RESET          177
#define XTAL_LOAD_CAP      183

// configure variables
int IQmult;
byte symbol[162];
byte c[11];                // encoded message
byte sym[170];             // symbol table 162
byte symt[170];            // symbol table temp
long long TempFreq;
unsigned long n1;          // encoded callsign
unsigned long m1;          // encodes locator
unsigned long debounce, DebounceDelay = 500000, XtalFreq = 25000000;
volatile unsigned int sycnt;
volatile byte ii,i,j;
byte msg_type,txTime2,temp = 1,calltype;
int nadd,nc,n,ntype;
char grid4[5],grid6[7],call1[7],cnt1;
unsigned long t1,ng,n2,cc1;
long MASK15=32767,ihash;

// Load WSPR symbol frequency offsets
int OffsetFreq[4] = {
  -219, // 0 Hz
  -73,  // 1.46 Hz
  73,  // 2.93 Hz
  219   // 4.39 Hz
};

const char SyncVec[162] = {
  1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
  1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0,
  0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0,
  0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0
};



void setup()
{
  Wire.begin(1);                     // join i2c bus (address = 1)

  XtalFreq = XtalFreq + CalFactor;

  // Set up push buttons
  pinMode(CalSet, INPUT);
  digitalWrite(CalSet, HIGH);        // internal pull-up enabled
  pinMode(TX, INPUT);
  digitalWrite(TX, HIGH);            // internal pull-up enabled
  pinMode(PTT,OUTPUT);
  digitalWrite(PTT, LOW);            // ensure antenna relay is in receive mode
  
  IQmult = 4;
  
  // Ensure input data is upper case
  for(i=0;i<13;i++)if(call2[i]>=97&&call2[i]<=122)call2[i]=call2[i]-32;
  for(i=0;i<7;i++)if(locator[i]>=97&&locator[i]<=122)locator[i]=locator[i]-32;

// WSPR message calculation
   wsprGenCode();  

// Initialize the Si5351
  Si5351_write(XTAL_LOAD_CAP, 0b11000000);      // Set crystal load to 10pF
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000110); // Enable CLK0 - CLK1 and CLK2 OFF
  Si5351_write(CLK0_CONTROL, 0b00001111);       // Set PLLA to CLK0, 8 mA output
  Si5351_write(CLK1_CONTROL, 0b00001111);       // Set PLLA to CLK1, 8 mA output
  Si5351_write(CLK2_CONTROL, 0b00101111);       // Set PLLB to CLK2, 8 mA output
  Si5351_write(PLL_RESET, 0b10100000);          // Reset PLLA and PLLB

// Set PLLA  and PLLB to 600 MHz
  si5351aSetPLL(SYNTH_PLL_A, 600000000);
  si5351aSetPLL(SYNTH_PLL_B, 600000000);  

// Set CLK0 to dial frequency 
  si5351aSetFreq2 (SYNTH_MS_0, (RXdialFreq [band] - IQwspr) * IQmult * 100);

// Set CLK1 to 25 MHz for calibration if pin 2 is set LOW  
  if(digitalRead(CalSet) == LOW) 
  {
    Si5351_write(CLK_ENABLE_CONTROL, 0b00000101); // Disable CLK0 and CLK2 - Enable CLK1
    si5351aSetFreq2 (SYNTH_MS_1,2500000000);
  }
}

void loop()
{
  if (digitalRead(TX) == LOW) // Ttansmit starts here:
  {
    Si5351_write(CLK_ENABLE_CONTROL, 0b00000011); // Disable CLK0 and CLK1 - Enable CLK2
    transmit();
  }
  delay(100);
}


//******************************************************************
void transmit()
{
  digitalWrite(PTT, HIGH);
  if(FreqHopTX == 1) // Enables in-band TX frequency hopping in incremental 15Hz steps
  {
    TXoffset = TXoffset + 15;
    if(TXoffset > 1590) TXoffset = 1415;
  }
  for (int count = 0; count < 162; count++)
  {
    si5351aSetFreq2(SYNTH_MS_2, (RXdialFreq [band] + TXoffset) * 100 + OffsetFreq[sym[count]]);
    delay(681);
  }
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000110); // Enable CLK0 - Disable CLK1 and CLK2
  while(digitalRead(TX) == LOW){delay(100);}    // Ensure TX is HIGH before proceeding
  if(calltype == 2)
  {
   msg_type = !msg_type;
   wsprGenCode();   
  }
  delay(10);
  digitalWrite(PTT, LOW);
}


//******************************************************************
//  Si5351 PLL processing
//******************************************************************
void si5351aSetPLL(int synth, long long PLLfreq)
{
  unsigned long long CalcTemp, a;
  unsigned long  b, c, p1, p2, p3;

  c = 0xFFFFF;  // Denominator derived from max bits 2^20

  a = PLLfreq / XtalFreq;
  CalcTemp = PLLfreq % XtalFreq;
  CalcTemp *= c;
  CalcTemp /= XtalFreq ;
  b = CalcTemp;  // Calculated numerator


  // Refer to Si5351 Register Map AN619 for following formula
  p3  = c;
  p2  = (128 * b) % c;
  p1  = 128 * a;
  p1 += (128 * b / c);
  p1 -= 512;

  // Write data to multisynth registers
  Si5351_write(synth, 0xFF);
  Si5351_write(synth + 1, 0xFF);
  Si5351_write(synth + 2, (p1 & 0x00030000) >> 16);
  Si5351_write(synth + 3, (p1 & 0x0000FF00) >> 8);
  Si5351_write(synth + 4, (p1 & 0x000000FF));
  Si5351_write(synth + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  Si5351_write(synth + 6, (p2 & 0x0000FF00) >> 8);
  Si5351_write(synth + 7, (p2 & 0x000000FF));

}



//******************************************************************
//  Si5351 Multisynch processing
//******************************************************************
void si5351aSetFreq2(int synth, unsigned long long freq)
{
  unsigned long long CalcTemp;
  unsigned long  b, c, p1, p2, p3;

  c = 0xFFFFF;  // Denominator derived from max bits 2^20

  long long a = 60000000000 / freq; 
  CalcTemp = 60000000000 % freq;
  CalcTemp *= c;
  CalcTemp /= freq ;
  b = CalcTemp;  // Calculated numerator


  // Refer to Si5351 Register Map AN619 for following formula
  p3  = c;
  p2  = (128 * b) % c;
  p1  = 128 * a;
  p1 += (128 * b / c);
  p1 -= 512;

  // Write data to multisynth registers
  Si5351_write(synth, 0xFF);
  Si5351_write(synth + 1, 0xFF);
  Si5351_write(synth + 2, (p1 & 0x00030000) >> 16);
  Si5351_write(synth + 3, (p1 & 0x0000FF00) >> 8);
  Si5351_write(synth + 4, (p1 & 0x000000FF));
  Si5351_write(synth + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  Si5351_write(synth + 6, (p2 & 0x0000FF00) >> 8);
  Si5351_write(synth + 7, (p2 & 0x000000FF));

}


//******************************************************************
//Write I2C data routine
//******************************************************************
uint8_t Si5351_write(uint8_t addr, uint8_t data)
{
  Wire.beginTransmission(Si5351A_addr);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}
//------------------------------------------------------------------------------------------------------


//******************************************************************
void wsprGenCode()
{
  for(i=0;i<13;i++){if(call2[i] == 47)calltype=2;};
  if(calltype == 2) type2();
  else
  {
  for(i=0;i<7;i++){call1[i] = call2[i]; };
  for(i=0;i<5;i++){
  grid4[i] = locator[i];
  };
  packcall();
  packgrid();
  n2=ng*128+ndbm+64;
  pack50();
  encode_conv();
  interleave_sync();
  }
}

//******************************************************************
void type2()
{
  if(msg_type == 0)
  {
  packpfx();
  ntype=ndbm + 1 + nadd;
  n2 = 128*ng + ntype + 64;
  pack50();
  encode_conv();
  interleave_sync();  
  }
  else
  {
    hash();
    for(ii=1;ii<6;ii++)
    {
      call1[ii-1]=locator[ii];
    };
    call1[5]=locator[0];
    packcall();
    ntype=-(ndbm+1);
    n2=128*ihash + ntype +64;
    pack50();
    encode_conv();
    interleave_sync();
  };

}

//******************************************************************
void packpfx()
{
  char pfx[3];
  int Len;
  int slash;

  for(i=0;i<7;i++)
  {
    call1[i]=0;
  };
  Len = strlen(call2);
  for(i=0;i<13;i++)
  {
    if(call2[i] == 47) slash = i;
  };
  if(call2[slash+2] == 0)
  {//single char add-on suffix
    for(i=0;i<slash;i++)
    {
      call1[i] = call2[i];
    };
    packcall();
    nadd=1;
    nc=int(call2[slash+1]);
    if(nc>=48 && nc<=57) n=nc-48;
    else if(nc>=65 && nc<=90) n=nc-65+10;
    else if (nc>=97 && nc<=122) n=nc-97+10;
    else n=38;
    ng=60000-32768+n;
  }
  else
    if(call2[slash+3] == 0)
    {
     for(i=0;i<slash;i++)
    {
      call1[i] = call2[i];
    };
    packcall();
    n=10*(int(call2[slash+1])-48) +  int(call2[slash+2])-48;
    nadd=1;
    ng=60000 + 26 + n;
    }
    else
    {
     for(i=0;i<slash;i++)
    {
     pfx[i] = call2[i];
    };
    if (slash == 2)
    {
     pfx[2] = pfx[1];
     pfx[1] = pfx[0];
     pfx[0] = ' ';
    };
    if (slash == 1)
    {
     pfx[2] = pfx[0];
     pfx[1] = ' ';
     pfx[0] = ' ';
    };
     ii=0;
     for(i=slash+1;i<Len;i++)
    {
      call1[ii] = call2[i];
      ii++;
     };
    packcall();
    ng=0;
    for(i=0;i<3;i++)
    {
     nc=int(pfx[i]);
    if(nc>=48 && nc<=57) n=nc-48;
    else if(nc>=65 && nc<=90) n=nc-65+10;
    else if (nc>=97 && nc<=122) n=nc-97+10;
    else n=36;     
    ng=37*ng+n;
    };
    nadd=0;
    if(ng >= 32768)
    {
     ng=ng-32768;
     nadd=1; 
    };
   }
 }

//******************************************************************
void packcall()
{
  // coding of callsign
  if (chr_normf(call1[2]) > 9) 
  {
    call1[5] = call1[4];
    call1[4] = call1[3]; 
    call1[3] = call1[2];
    call1[2] = call1[1];
    call1[1] = call1[0];
    call1[0] = ' ';
  }

  n1=chr_normf(call1[0]);
  n1=n1*36+chr_normf(call1[1]);
  n1=n1*10+chr_normf(call1[2]);
  n1=n1*27+chr_normf(call1[3])-10;
  n1=n1*27+chr_normf(call1[4])-10;
  n1=n1*27+chr_normf(call1[5])-10;
}

//******************************************************************
void packgrid()
{
  // coding of grid4
  ng=179-10*(chr_normf(grid4[0])-10)-chr_normf(grid4[2]);
  ng=ng*180+10*(chr_normf(grid4[1])-10)+chr_normf(grid4[3]);
}

//******************************************************************
void pack50()
{
  // merge coded callsign into message array c[]
  t1=n1;
  c[0]= t1 >> 20;
  t1=n1;
  c[1]= t1 >> 12;
  t1=n1;
  c[2]= t1 >> 4;
  t1=n1;
  c[3]= t1 << 4;
  t1=n2;
  c[3]= c[3] + ( 0x0f & t1 >> 18);
  t1=n2;
  c[4]= t1 >> 10;
  t1=n2;
  c[5]= t1 >> 2;
  t1=n2;
  c[6]= t1 << 6;
}

//******************************************************************
//void hash(string,len,ihash)
void hash()
{
  int Len;
  uint32_t jhash;
  int *pLen = &Len;
  Len = strlen(call2); 
  byte IC[12];
  byte *pIC = IC;
  for (i=0;i<12;i++)
  {
    pIC + 1;
    &IC[i];
  }
  uint32_t Val = 146;
  uint32_t *pVal = &Val;
  for(i=0;i<Len;i++)
  {
    IC[i] = int(call2[i]);
  };
  jhash=nhash_(pIC,pLen,pVal);
  ihash=jhash&MASK15;
  return;
}
//******************************************************************
// normalize characters 0..9 A..Z Space in order 0..36
char chr_normf(char bc ) 
{
  char cc=36;
  if (bc >= '0' && bc <= '9') cc=bc-'0';
  if (bc >= 'A' && bc <= 'Z') cc=bc-'A'+10;
  if (bc >= 'a' && bc <= 'z') cc=bc-'a'+10;  
  if (bc == ' ' ) cc=36;

  return(cc);
}


//******************************************************************
// convolutional encoding of message array c[] into a 162 bit stream
void encode_conv()
{
  int bc=0;
  int cnt=0;
  int cc;
  unsigned long sh1=0;

  cc=c[0];

  for (int i=0; i < 81;i++) {
    if (i % 8 == 0 ) {
      cc=c[bc];
      bc++;
    }
    if (cc & 0x80) sh1=sh1 | 1;

    symt[cnt++]=parity(sh1 & 0xF2D05351);
    symt[cnt++]=parity(sh1 & 0xE4613C47);

    cc=cc << 1;
    sh1=sh1 << 1;
  }
}

//******************************************************************
byte parity(unsigned long li)
{
  byte po = 0;
  while(li != 0)
  {
    po++;
    li&= (li-1);
  }
  return (po & 1);
}

//******************************************************************
// interleave reorder the 162 data bits and and merge table with the sync vector
void interleave_sync()
{
  int ii,ij,b2,bis,ip;
  ip=0;

  for (ii=0;ii<=255;ii++) {
    bis=1;
    ij=0;
    for (b2=0;b2 < 8 ;b2++) {
      if (ii & bis) ij= ij | (0x80 >> b2);
      bis=bis << 1;
    }
    if (ij < 162 ) {
      sym[ij]= SyncVec[ij] +2*symt[ip];
      ip++;
    }
  }
}


//_____________________________________________________________________________
// Note: The parts of the routine that follows is used for WSPR type 2 callsigns 
//(i.e. GM/W3PM) to generate the required hash code.
//_____________________________________________________________________________

/*
-------------------------------------------------------------------------------
lookup3.c, by Bob Jenkins, May 2006, Public Domain.

These are functions for producing 32-bit hashes for hash table lookup.
hashword(), hashlittle(), hashlittle2(), hashbig(), mix(), and final() 
are externally useful functions.  Routines to test the hash are included 
if SELF_TEST is defined.  You can use this free for any purpose.  It's in
the public domain.  It has no warranty.

You probably want to use hashlittle().  hashlittle() and hashbig()
hash byte arrays.  hashlittle() is is faster than hashbig() on
little-endian machines.  Intel and AMD are little-endian machines.
On second thought, you probably want hashlittle2(), which is identical to
hashlittle() except it returns two 32-bit hashes for the price of one.  
You could implement hashbig2() if you wanted but I haven't bothered here.

If you want to find a hash of, say, exactly 7 integers, do
  a = i1;  b = i2;  c = i3;
  mix(a,b,c);
  a += i4; b += i5; c += i6;
  mix(a,b,c);
  a += i7;
  final(a,b,c);
then use c as the hash value.  If you have a variable length array of
4-byte integers to hash, use hashword().  If you have a byte array (like
a character string), use hashlittle().  If you have several byte arrays, or
a mix of things, see the comments above hashlittle().  

Why is this so big?  I read 12 bytes at a time into 3 4-byte integers, 
then mix those integers.  This is fast (you can do a lot more thorough
mixing with 12*3 instructions on 3 integers than you can with 3 instructions
on 1 byte), but shoehorning those bytes into integers efficiently is messy.
-------------------------------------------------------------------------------
*/

//#define SELF_TEST 1

//#include <stdio.h>      /* defines printf for tests */
//#include <time.h>       /* defines time_t for timings in the test */
//#ifdef Win32
//#include "win_stdint.h"	/* defines uint32_t etc */
//#else
//#include <stdint.h>	/* defines uint32_t etc */
//#endif

//#include <sys/param.h>  /* attempt to define endianness */
//#ifdef linux
//# include <endian.h>    /* attempt to define endianness */
//#endif

#define HASH_LITTLE_ENDIAN 1

#define hashsize(n) ((uint32_t)1<<(n))
#define hashmask(n) (hashsize(n)-1)
#define rot(x,k) (((x)<<(k)) | ((x)>>(32-(k))))

/*
-------------------------------------------------------------------------------
mix -- mix 3 32-bit values reversibly.

This is reversible, so any information in (a,b,c) before mix() is
still in (a,b,c) after mix().

If four pairs of (a,b,c) inputs are run through mix(), or through
mix() in reverse, there are at least 32 bits of the output that
are sometimes the same for one pair and different for another pair.
This was tested for:
* pairs that differed by one bit, by two bits, in any combination
  of top bits of (a,b,c), or in any combination of bottom bits of
  (a,b,c).
* "differ" is defined as +, -, ^, or ~^.  For + and -, I transformed
  the output delta to a Gray code (a^(a>>1)) so a string of 1's (as
  is commonly produced by subtraction) look like a single 1-bit
  difference.
* the base values were pseudorandom, all zero but one bit set, or 
  all zero plus a counter that starts at zero.

Some k values for my "a-=c; a^=rot(c,k); c+=b;" arrangement that
satisfy this are
    4  6  8 16 19  4
    9 15  3 18 27 15
   14  9  3  7 17  3
Well, "9 15 3 18 27 15" didn't quite get 32 bits diffing
for "differ" defined as + with a one-bit base and a two-bit delta.  I
used http://burtleburtle.net/bob/hash/avalanche.html to choose 
the operations, constants, and arrangements of the variables.

This does not achieve avalanche.  There are input bits of (a,b,c)
that fail to affect some output bits of (a,b,c), especially of a.  The
most thoroughly mixed value is c, but it doesn't really even achieve
avalanche in c.

This allows some parallelism.  Read-after-writes are good at doubling
the number of bits affected, so the goal of mixing pulls in the opposite
direction as the goal of parallelism.  I did what I could.  Rotates
seem to cost as much as shifts on every machine I could lay my hands
on, and rotates are much kinder to the top and bottom bits, so I used
rotates.
-------------------------------------------------------------------------------
*/
#define mix(a,b,c) \
{ \
  a -= c;  a ^= rot(c, 4);  c += b; \
  b -= a;  b ^= rot(a, 6);  a += c; \
  c -= b;  c ^= rot(b, 8);  b += a; \
  a -= c;  a ^= rot(c,16);  c += b; \
  b -= a;  b ^= rot(a,19);  a += c; \
  c -= b;  c ^= rot(b, 4);  b += a; \
}

/*
-------------------------------------------------------------------------------
final -- final mixing of 3 32-bit values (a,b,c) into c

Pairs of (a,b,c) values differing in only a few bits will usually
produce values of c that look totally different.  This was tested for
* pairs that differed by one bit, by two bits, in any combination
  of top bits of (a,b,c), or in any combination of bottom bits of
  (a,b,c).
* "differ" is defined as +, -, ^, or ~^.  For + and -, I transformed
  the output delta to a Gray code (a^(a>>1)) so a string of 1's (as
  is commonly produced by subtraction) look like a single 1-bit
  difference.
* the base values were pseudorandom, all zero but one bit set, or 
  all zero plus a counter that starts at zero.

These constants passed:
 14 11 25 16 4 14 24
 12 14 25 16 4 14 24
and these came close:
  4  8 15 26 3 22 24
 10  8 15 26 3 22 24
 11  8 15 26 3 22 24
-------------------------------------------------------------------------------
*/
#define final(a,b,c) \
{ \
  c ^= b; c -= rot(b,14); \
  a ^= c; a -= rot(c,11); \
  b ^= a; b -= rot(a,25); \
  c ^= b; c -= rot(b,16); \
  a ^= c; a -= rot(c,4);  \
  b ^= a; b -= rot(a,14); \
  c ^= b; c -= rot(b,24); \
}

/*
-------------------------------------------------------------------------------
hashlittle() -- hash a variable-length key into a 32-bit value
  k       : the key (the unaligned variable-length array of bytes)
  length  : the length of the key, counting by bytes
  initval : can be any 4-byte value
Returns a 32-bit value.  Every bit of the key affects every bit of
the return value.  Two keys differing by one or two bits will have
totally different hash values.

The best hash table sizes are powers of 2.  There is no need to do
mod a prime (mod is sooo slow!).  If you need less than 32 bits,
use a bitmask.  For example, if you need only 10 bits, do
  h = (h & hashmask(10));
In which case, the hash table should have hashsize(10) elements.

If you are hashing n strings (uint8_t **)k, do it like this:
  for (i=0, h=0; i<n; ++i) h = hashlittle( k[i], len[i], h);

By Bob Jenkins, 2006.  bob_jenkins@burtleburtle.net.  You may use this
code any way you wish, private, educational, or commercial.  It's free.

Use for hash table lookup, or anything where one collision in 2^^32 is
acceptable.  Do NOT use for cryptographic purposes.
-------------------------------------------------------------------------------
*/

//uint32_t hashlittle( const void *key, size_t length, uint32_t initval)
#ifdef STDCALL
uint32_t __stdcall NHASH( const void *key, size_t *length0, uint32_t *initval0)
#else
uint32_t nhash_( const void *key, int *length0, uint32_t *initval0)
#endif
{
  uint32_t a,b,c;                                          /* internal state */
  size_t length;
  uint32_t initval;
  union { const void *ptr; size_t i; } u;     /* needed for Mac Powerbook G4 */

  length=*length0;
  initval=*initval0;

  /* Set up the internal state */
  a = b = c = 0xdeadbeef + ((uint32_t)length) + initval;

  u.ptr = key;
  if (HASH_LITTLE_ENDIAN && ((u.i & 0x3) == 0)) {
    const uint32_t *k = (const uint32_t *)key;         /* read 32-bit chunks */
    const uint8_t  *k8;

    k8=0;                                     //Silence compiler warning
    /*------ all but last block: aligned reads and affect 32 bits of (a,b,c) */
    while (length > 12)
    {
      a += k[0];
      b += k[1];
      c += k[2];
      mix(a,b,c);
      length -= 12;
      k += 3;
    }

    /*----------------------------- handle the last (probably partial) block */
    /* 
     * "k[2]&0xffffff" actually reads beyond the end of the string, but
     * then masks off the part it's not allowed to read.  Because the
     * string is aligned, the masked-off tail is in the same word as the
     * rest of the string.  Every machine with memory protection I've seen
     * does it on word boundaries, so is OK with this.  But VALGRIND will
     * still catch it and complain.  The masking trick does make the hash
     * noticably faster for short strings (like English words).
     */
#ifndef VALGRIND

    switch(length)
    {
    case 12: c+=k[2]; b+=k[1]; a+=k[0]; break;
    case 11: c+=k[2]&0xffffff; b+=k[1]; a+=k[0]; break;
    case 10: c+=k[2]&0xffff; b+=k[1]; a+=k[0]; break;
    case 9 : c+=k[2]&0xff; b+=k[1]; a+=k[0]; break;
    case 8 : b+=k[1]; a+=k[0]; break;
    case 7 : b+=k[1]&0xffffff; a+=k[0]; break;
    case 6 : b+=k[1]&0xffff; a+=k[0]; break;
    case 5 : b+=k[1]&0xff; a+=k[0]; break;
    case 4 : a+=k[0]; break;
    case 3 : a+=k[0]&0xffffff; break;
    case 2 : a+=k[0]&0xffff; break;
    case 1 : a+=k[0]&0xff; break;
    case 0 : return c;              /* zero length strings require no mixing */
    }

#else /* make valgrind happy */

    k8 = (const uint8_t *)k;
    switch(length)
    {
    case 12: c+=k[2]; b+=k[1]; a+=k[0]; break;
    case 11: c+=((uint32_t)k8[10])<<16;  /* fall through */
    case 10: c+=((uint32_t)k8[9])<<8;    /* fall through */
    case 9 : c+=k8[8];                   /* fall through */
    case 8 : b+=k[1]; a+=k[0]; break;
    case 7 : b+=((uint32_t)k8[6])<<16;   /* fall through */
    case 6 : b+=((uint32_t)k8[5])<<8;    /* fall through */
    case 5 : b+=k8[4];                   /* fall through */
    case 4 : a+=k[0]; break;
    case 3 : a+=((uint32_t)k8[2])<<16;   /* fall through */
    case 2 : a+=((uint32_t)k8[1])<<8;    /* fall through */
    case 1 : a+=k8[0]; break;
    case 0 : return c;
    }

#endif /* !valgrind */

  } else if (HASH_LITTLE_ENDIAN && ((u.i & 0x1) == 0)) {
    const uint16_t *k = (const uint16_t *)key;         /* read 16-bit chunks */
    const uint8_t  *k8;

    /*--------------- all but last block: aligned reads and different mixing */
    while (length > 12)
    {
      a += k[0] + (((uint32_t)k[1])<<16);
      b += k[2] + (((uint32_t)k[3])<<16);
      c += k[4] + (((uint32_t)k[5])<<16);
      mix(a,b,c);
      length -= 12;
      k += 6;
    }

    /*----------------------------- handle the last (probably partial) block */
    k8 = (const uint8_t *)k;
    switch(length)
    {
    case 12: c+=k[4]+(((uint32_t)k[5])<<16);
             b+=k[2]+(((uint32_t)k[3])<<16);
             a+=k[0]+(((uint32_t)k[1])<<16);
             break;
    case 11: c+=((uint32_t)k8[10])<<16;     /* fall through */
    case 10: c+=k[4];
             b+=k[2]+(((uint32_t)k[3])<<16);
             a+=k[0]+(((uint32_t)k[1])<<16);
             break;
    case 9 : c+=k8[8];                      /* fall through */
    case 8 : b+=k[2]+(((uint32_t)k[3])<<16);
             a+=k[0]+(((uint32_t)k[1])<<16);
             break;
    case 7 : b+=((uint32_t)k8[6])<<16;      /* fall through */
    case 6 : b+=k[2];
             a+=k[0]+(((uint32_t)k[1])<<16);
             break;
    case 5 : b+=k8[4];                      /* fall through */
    case 4 : a+=k[0]+(((uint32_t)k[1])<<16);
             break;
    case 3 : a+=((uint32_t)k8[2])<<16;      /* fall through */
    case 2 : a+=k[0];
             break;
    case 1 : a+=k8[0];
             break;
    case 0 : return c;                     /* zero length requires no mixing */
    }

  } else {                        /* need to read the key one byte at a time */
    const uint8_t *k = (const uint8_t *)key;

    /*--------------- all but the last block: affect some 32 bits of (a,b,c) */
    while (length > 12)
    {
      a += k[0];
      a += ((uint32_t)k[1])<<8;
      a += ((uint32_t)k[2])<<16;
      a += ((uint32_t)k[3])<<24;
      b += k[4];
      b += ((uint32_t)k[5])<<8;
      b += ((uint32_t)k[6])<<16;
      b += ((uint32_t)k[7])<<24;
      c += k[8];
      c += ((uint32_t)k[9])<<8;
      c += ((uint32_t)k[10])<<16;
      c += ((uint32_t)k[11])<<24;
      mix(a,b,c);
      length -= 12;
      k += 12;
    }

    /*-------------------------------- last block: affect all 32 bits of (c) */
    switch(length)                   /* all the case statements fall through */
    {
    case 12: c+=((uint32_t)k[11])<<24;
    case 11: c+=((uint32_t)k[10])<<16;
    case 10: c+=((uint32_t)k[9])<<8;
    case 9 : c+=k[8];
    case 8 : b+=((uint32_t)k[7])<<24;
    case 7 : b+=((uint32_t)k[6])<<16;
    case 6 : b+=((uint32_t)k[5])<<8;
    case 5 : b+=k[4];
    case 4 : a+=((uint32_t)k[3])<<24;
    case 3 : a+=((uint32_t)k[2])<<16;
    case 2 : a+=((uint32_t)k[1])<<8;
    case 1 : a+=k[0];
             break;
    case 0 : return c;
    }
  }

  final(a,b,c);
  return c;
}

//uint32_t __stdcall NHASH(const void *key, size_t length, uint32_t initval)


