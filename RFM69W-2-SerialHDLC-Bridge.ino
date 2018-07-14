#define SERIAL_SPEED 115200

#include <util/crc16.h>

/// HDLC /////////////////////////////////////////////////////////
/* a protocol for serial communication featuring frames, checksum and auto-escaping of characters.
    datasheet: http://tools.ietf.org/html/rfc1662

    send:
        Serial.begin(57600);        // open serialport
        hdlc.frameOpen();       // framestart
        hdlc.frameWrite(byte data); //repeat as often as you want
        hdlc.frameClose();      // frame-ending (&checksum) and flush

    receive:
        uint8_t incoming_msg[80];
        if (uint8_t msg_size = hdlc.poll())
        {
            for (uint8_t ivar=0; ivar < size_msg; ++ivar )
            {
                incoming_msg[ivar] = hdlc.getMsg(ivar);
            };
        };
        hdlc.clearMsg();

*/

class HDLC
{
private:

    static constexpr uint8_t  HDLC_FLAG                   = (0x7e);
    static constexpr uint8_t  HDLC_ESCAPE                 = (0x7d);
    static constexpr uint8_t  HDLC_FLAG_ESC               = (0x5e);
    static constexpr uint8_t  HDLC_ESCAPE_ESC             = (0x5d);
    static constexpr uint8_t  HDLC_ESCAPE_MASK            = (0x20);
    static constexpr uint16_t HDLC_CRCINIT                = (0xffff);
    static constexpr uint16_t HDLC_CRCFINALXOR            = (0x0);
    static constexpr uint16_t HDLC_CRCGOOD                = (0xf0b8);

    static constexpr uint16_t HDLC_MAX_FRAME_LENGTH       = (128);
    static constexpr uint16_t HDLC_INPUT_BUFFER_SIZE      = (HDLC_MAX_FRAME_LENGTH);


    uint16_t crcIteration(const uint16_t crc, const uint8_t _b)
    {
      return _crc_xmodem_update( crc, _b );
    }

    uint16_t crc16;
    uint8_t  msg_now, msg_received, msg[HDLC_INPUT_BUFFER_SIZE];

public:

    HDLC(void): crc16(0), msg_now(0), msg_received(0)
    {
        for (uint8_t loopvar = 0; loopvar < HDLC_INPUT_BUFFER_SIZE; ++loopvar)
        {
            msg[loopvar] = 0;
        };

    };


// TODO: not ideal, but ok for now


    void frameOpen(void)
    {
        Serial.write(HDLC_FLAG);
        crc16 = HDLC_CRCINIT;
    };

    void frameWrite(const uint8_t _b)
    {

        crc16 = crcIteration(crc16,_b);

        uint8_t bw;

        if (_b == HDLC_FLAG)
        {
            Serial.write(HDLC_ESCAPE);
            bw = HDLC_FLAG_ESC;
        }
        else if (_b == HDLC_ESCAPE)
        {
            Serial.write(HDLC_ESCAPE);
            bw = HDLC_ESCAPE_ESC;
        }
        else bw = _b;

        Serial.write(bw);

    };

    uint16_t frameClose(void)
    {
//        uint16_t _crc = ~crc16;
        uint16_t _crc = crc16 ^ HDLC_CRCFINALXOR;

        frameWrite((_crc>>8)& 0xFF);
        frameWrite((_crc>>0)& 0xFF);
        

        Serial.write(HDLC_FLAG);
        Serial.flush();
    };


    uint8_t poll()
    {
        if (msg_received < 3)  // shortcut, nothing to analyze
        {

//            uint16_t serdelay = 9000000 / SERIAL_SPEED;
            while (Serial.available())
            {
                uint8_t _b  = uint8_t(Serial.read());

                if (msg_received == 0)
                {
                    if (_b==HDLC_FLAG) msg_received = 1;
                    if (!Serial.available()) delay(1);
                    continue;
                };
                if ((msg_received == 2)&&(_b==HDLC_FLAG))  // message complete
                {
                    msg_received = 3;
                    break;
                };
                if (msg_received == 1)
                {
                    if (_b==HDLC_FLAG)  continue; // startflag found, do nothing
                    else                msg_received = 2; // >1 other character after specialflag found, normal message buffering
                };

                if (msg_received == 2)
                {
                    if (_b == HDLC_ESCAPE)
                    {
                        uint8_t wstop = 0;
                        while(!Serial.available())
                        {
                            //ps.sleep ( 0 );
                            delay(1);
                            if (wstop++ > 10) break;
                        };
                        _b = uint8_t(Serial.read());
                        if (_b == HDLC_FLAG_ESC)     _b = HDLC_FLAG;
                        if (_b == HDLC_ESCAPE_ESC)   _b = HDLC_ESCAPE;
                    };
                    msg[msg_now] = _b;
                    msg_now ++;
                };
                if (!Serial.available())   delay(1); //ps.sleep ( 0 );
            };
        };

        if (msg_received == 3)
        {
            msg_now -= 2; // cut away sync-bytes

            uint16_t crcT = HDLC_CRCINIT;
            for (uint8_t ivar = 0; ivar < msg_now; ++ivar)
            {
                crcT = crcIteration(crcT,msg[ivar]);
            };
            //crcT = ~crcT;
            crcT = crcT ^ HDLC_CRCFINALXOR;
            //uint16_t crcM = msg[msg_now] | (msg[msg_now+1]<<8);
            uint16_t crcM = (msg[msg_now]<<8) | (msg[msg_now+1]);

            if (crcM != crcT)
            {
                clearMsg();
                return 0;
            };

            msg_received = 4;
            return msg_now;
        };

        if (msg_received > 2)  return msg_now;
        else                   return 0 ;
    };



    uint8_t getMsg(const uint8_t position)
    {
        uint8_t _b;
        if (position > (HDLC_INPUT_BUFFER_SIZE - 1)) _b = 0;
        else                                         _b = msg[position];

        return _b;
    };

    void clearMsg()
    {
        msg_received = 0;
        msg_now = 0;
    };

};



HDLC hdlc;


//#define DEBUGSENSOR


//#define HWUNO
#define HWMOTE
//#define HWMINI
#define HWRFM69W
//#define HWASK433

#ifdef HWUNO
  #define STATUS_LED LED_BUILTIN
#endif

#ifdef HWMOTE  
  #define STATUS_LED 9
#endif

#ifdef HWMINI
  #define STATUS_LED LED_BUILTIN
#endif

#ifdef DEBUGSENSOR
  #include <SoftwareSerial.h>
  SoftwareSerial mySerial(4,5); // RX, TX
#endif

#include <RH_RF69.h>

//unsigned long c = 0;
class RH_RF69_koug : public RH_RF69
{
  protected:
  void readFifo()
  {
//    Serial.println("readfifo!");

//    unsigned long a = micros();
        ATOMIC_BLOCK_START;
    digitalWrite(_slaveSelectPin, LOW);
    SPI.transfer(RH_RF69_REG_00_FIFO); // Send the start address with the write mask off
    uint8_t payloadlen = SPI.transfer(0); // First byte is payload len (counting the headers)
    //if (payloadlen <= RH_RF69_MAX_ENCRYPTABLE_PAYLOAD_LEN &&
 //payloadlen >= RH_RF69_HEADER_LEN)
    {
  _rxHeaderTo = SPI.transfer(0);
  // Check addressing
//  if (_promiscuous ||
//      _rxHeaderTo == _thisAddress ||
//      _rxHeaderTo == RH_BROADCAST_ADDRESS)
  {
      // Get the rest of the headers
      _rxHeaderFrom  = SPI.transfer(0);
      _rxHeaderId    = SPI.transfer(0);
      _rxHeaderFlags = SPI.transfer(0);
      // And now the real payload
      uint8_t maxb = (payloadlen - RH_RF69_HEADER_LEN);
      for (_bufLen = 0; _bufLen < maxb; _bufLen++)
      {
//        _buf[_bufLen] = SPI.transfer(0);
        SPDR = 0;   // this and next 3 lines do the same as the above commented, only faster
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ;
        _buf[_bufLen] = SPDR; 
      }
      _rxGood++;
      _rxBufValid = true;
  }
    }
    digitalWrite(_slaveSelectPin, HIGH);
    ATOMIC_BLOCK_END;
    // Any junk remaining in the FIFO will be cleared next time we go to receive mode.
//    c = micros() - a;
  }

};
      
#ifdef HWRFM69W
  RH_RF69_koug drvrf69;
#endif  
  


typedef struct header {
  uint8_t proto_ver : 4;
  uint8_t proto_type : 4;
  uint16_t ssid;
  uint16_t dsid;
  uint16_t msgseq;
  uint8_t qos : 4;
  uint8_t :4;
  uint8_t ttl;
  uint8_t frg : 4;
  uint8_t tfrg : 4;
  uint16_t mtu : 11;
  uint16_t  : 5;
  uint16_t nexthdr;
} header;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(SERIAL_SPEED);
  pinMode( STATUS_LED, OUTPUT );
  digitalWrite( STATUS_LED, LOW);

#ifdef DEBUGSENSOR
  mySerial.begin( 9600 );
  mySerial.println( "Starting up!");
#endif

#ifdef HWRFM69W
  drvrf69.init();
  drvrf69.setFrequency(433.0);
//  uint8_t syncwords[] = { 0x2a, 0xd4 };  // def is 0x2d, 0xd4
//  driver.setSyncWords(syncwords, sizeof(syncwords)); // set to something else / different network
#endif    
}

//unsigned long a = micros();
//unsigned long b = 0;
//int diff = 0;

void fwdmsg_torf(uint8_t msg_size)
{
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];      
  uint8_t buflen = sizeof(buf);
  uint8_t ivar = 0;
  header *reshead = (header *)buf;
  
#ifdef DEBUGSENSOR  
  mySerial.print(F("Got hdlc msg, size is: "));
  mySerial.println(msg_size);
#endif  
  
  for (ivar = 0; ivar < msg_size; ++ivar )
  {
      buf[ivar] = hdlc.getMsg(ivar);
  };

  
  hdlc.clearMsg();

#ifdef DEBUGSENSOR  

  mySerial.print(F("HDLC received bytes: "));
  mySerial.println(msg_size);
  mySerial.print(F("HDLC received:<"));
//  for( uint8_t i = msgbuf_pos ; i < endbuf_pos ; i++ )
//  {
//    mySerial.print((char)(messagebuf[i]));
//  }
  mySerial.println(F(">"));
  mySerial.print(F("FRG is: "));
  mySerial.print(reshead->frg);
  mySerial.print(F(" TFRG is: "));
  mySerial.println(reshead->tfrg);
#endif

  reshead->ttl -= 1;
  drvrf69.send( buf, msg_size );

}

void fwdmsg_toser(void)
{
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];      
    uint8_t buflen = sizeof(buf);
    char moufaheader[] = "#SENSDATA#";

//    Serial.println();
//    Serial.print("Receive delay is: ");
//    Serial.println(c);
    
    if (drvrf69.recv(buf, &buflen))
    {
      if( buf[0] != '<' )
      {
        header *header_ptr;

        header_ptr = (header *)buf;
        header_ptr->ttl -= 1;
        
 
        hdlc.frameOpen();
        for( int i=0; i<buflen; i++ )
        {
          hdlc.frameWrite( buf[i] );
        }
        hdlc.frameClose();
      }
      else
      {
        int lastrssi = drvrf69.lastRssi();
        char extrathings[10];
        int extrathings_len; 
        sprintf(extrathings,"/RS0:%d",lastrssi);
        extrathings_len = strlen(extrathings);

        // Message with a good checksum received, dump it.
        //driver.printBuffer("Got:", buf, buflen);
        //Serial.println(buf);
//        buf[buflen-2] = 0; // adding end of string 
        hdlc.frameOpen();
        for( int i = 0 ; i < 10; i++ )
        {
          hdlc.frameWrite( moufaheader[i] );
        }
        for( int i = 1; i < buflen-2 ; i++ )
        {
          hdlc.frameWrite( buf[i] );
        }

        for( int i = 0 ; i < extrathings_len ; i++ )
        {
          hdlc.frameWrite(extrathings[i]);        
        }
//        Serial.println( (char *)(buf+1) );
        hdlc.frameClose();
      }
    }
}


void loop() {
  // put your main code here, to run repeatedly:
  
  if( uint8_t seravail = Serial.available() )
  {
#ifdef DEBUGSENSOR
    mySerial.print(F("Serial available got: "));
    mySerial.println(seravail);
#endif
    uint8_t msg_size = hdlc.poll();
#ifdef DEBUGSENSOR    
    mySerial.println(F("Got HDLC msg!"));
#endif    
    if( msg_size <= RH_RF69_MAX_MESSAGE_LEN && msg_size > 0 )
    {
      digitalWrite( STATUS_LED, HIGH);
      fwdmsg_torf( msg_size );
      digitalWrite( STATUS_LED, LOW);
    }
#ifdef DEBUGSENSOR
    else
    {
      mySerial.print(F("Ignored message from hdlc size:"));
      mySerial.println(msg_size);    
    }
#endif    
  }
#ifdef DEBUGSENSOR
  else
  {
//    mySerial.print("M");
  }
#endif  
  if ( drvrf69.available() )
  {
#ifdef DEBUGSENSOR      
    mySerial.println(F("Got RF msg!"));  
#endif    

    digitalWrite( STATUS_LED, HIGH); 
    fwdmsg_toser();
    digitalWrite( STATUS_LED, LOW);   
  }    
}


