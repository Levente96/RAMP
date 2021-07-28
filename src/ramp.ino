/**
   Copyright 2020 Levente Csóka
   v4.0.0
**/
#include <BlockDriver.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <SysCall.h>
#include <Scheduler.h>
#include "ramp.h"
#include <SPI.h>
#include <Wire.h>


#define ABORT_ON_OVERRUN 1
#define USE_SHARED_SPI 0
const uint32_t   LOG_INTERVAL_USEC   = 1250;
const uint8_t    SD_CS_PIN           = 4;
const int8_t     ERROR_LED_PIN       = 10;
const int8_t     STATUS_LED_PIN      = 9;
const uint32_t   FILE_BLOCK_COUNT    = 256000;
const uint8_t    BUFFER_BLOCK_COUNT  = 10;

SdFat            sd;
SdBaseFile       binFile;

const uint16_t   DATA_DIM            = (512 - 4)/sizeof(data_t);
const uint16_t   FILL_DIM            = 512 - 4 - DATA_DIM*sizeof(data_t);

int              status_blink        = 0;

struct block_t
{
    uint16_t  count;
    uint16_t  overrun;
    data_t    data[DATA_DIM];
    uint8_t   fill[FILL_DIM];
};

#define error(msg) {sd.errorPrint(&Serial1, F(msg));fatalBlink();}

#ifdef __arm__
    extern "C" char* sbrk(int incr);
#else  // __ARM__
    extern char *__brkval;
#endif  // __arm__
 
int freeMemory() {
    char top;
#ifdef __arm__
    return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
    return &top - __brkval;
#else  // __arm__
    return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

void fatalBlink()
{
    while (true)
    {
        if (ERROR_LED_PIN >= 0)
        {
            digitalWrite(ERROR_LED_PIN, HIGH);
            delay(200);
            digitalWrite(ERROR_LED_PIN, LOW);
            delay(200);
        }
    }
}

void createBinFile(String fname)
{
    const uint32_t ERASE_SIZE = 262144L;
    uint32_t bgnBlock, endBlock;

    if (!binFile.createContiguous(fname.c_str(), 512 * FILE_BLOCK_COUNT))
    {
        error("createContiguous failed");
    }
    if (!binFile.contiguousRange(&bgnBlock, &endBlock))
    {
        error("contiguousRange failed");
    }
    uint32_t bgnErase = bgnBlock;
    uint32_t endErase;
    while (bgnErase < endBlock)
    {
        endErase = bgnErase + ERASE_SIZE;
        if (endErase > endBlock)
        {
            endErase = endBlock;
        }
        if (!sd.card()->erase(bgnErase, endErase))
        {
            error("erase failed");
        }
        bgnErase = endErase + 1;
    }
}

void logData(String fname)
{
    createBinFile(fname);
    recordBinFile();
}

void recordBinFile()
{
    const uint8_t QUEUE_DIM = BUFFER_BLOCK_COUNT + 1;
    const uint8_t QUEUE_LAST = QUEUE_DIM - 1;
    block_t       block[BUFFER_BLOCK_COUNT - 1];
    block_t*      curBlock = 0;
    block_t*      emptyStack[BUFFER_BLOCK_COUNT];
    uint8_t       emptyTop;
    uint8_t       minTop;
    block_t*      fullQueue[QUEUE_DIM];
    uint8_t       fullHead = 0;
    uint8_t       fullTail = 0;
    emptyStack[0] = (block_t*)sd.vol()->cacheClear();
    if (emptyStack[0] == 0)
    {
        error("cacheClear failed");
    }
    for (int i = 1; i < BUFFER_BLOCK_COUNT; i++)
    {
        emptyStack[i] = &block[i - 1];
    }
    emptyTop = BUFFER_BLOCK_COUNT;
    minTop = BUFFER_BLOCK_COUNT;
    if (!sd.card()->writeStart(binFile.firstBlock()))
    {
      error("writeStart failed");
    }
    FreeStack();
    bool     closeFile    = false;
    uint32_t bn           = 0;
    uint32_t maxLatency   = 0;
    uint32_t overrun      = 0;
    uint32_t overrunTotal = 0;
    uint32_t logTime      = micros();
  
    while(1)
    {
        uint32_t before = micros();
        logTime += LOG_INTERVAL_USEC;
    
        if (Serial1.available())
        {
            String cmd = Serial1.readString();
            String stop_cmd = "STOP";
            if(strcmp(cmd.c_str(), "STOP") == 0)
            {
                Serial1.println("ACK");
                closeFile = true;
            }
            else
            {
                Serial1.print("Unknown command\"");
                Serial1.print(cmd);
                Serial1.println("\", try 'STOP'");
                Serial1.println(cmd.length());
                Serial1.println(stop_cmd.length());
            }
        }
    
        if (closeFile)
        {
            if (curBlock != 0)
            {
                fullQueue[fullHead] = curBlock;
                fullHead = fullHead < QUEUE_LAST ? fullHead + 1 : 0;
                curBlock = 0;
            }
        } else
        {
            if (curBlock == 0 && emptyTop != 0)
            {
                curBlock = emptyStack[--emptyTop];
                if (emptyTop < minTop)
                {
                    minTop = emptyTop;
                }
                curBlock->count = 0;
                curBlock->overrun = overrun;
                overrun = 0;
            }
            status_blink++;
            if(status_blink==0)
            {
                digitalWrite(STATUS_LED_PIN, HIGH);
            }
            if(status_blink==500)
            {
                digitalWrite(STATUS_LED_PIN, LOW);
            }
            if(status_blink>=1000)
            {
                status_blink = -1;
            }

            if (curBlock == 0)
            {
                overrun++;
                overrunTotal++;
                if (ERROR_LED_PIN >= 0)
                {
                    digitalWrite(ERROR_LED_PIN, HIGH);
                }
                #if ABORT_ON_OVERRUN
                    Serial.println(F("Overrun abort"));
                    break;
                #endif  // ABORT_ON_OVERRUN
            } else
            {
                acquireData(&curBlock->data[curBlock->count++]);
                Serial.print("Free memory: ");
                Serial.println(freeMemory());
                if (curBlock->count == DATA_DIM)
                {
                    fullQueue[fullHead] = curBlock;
                    fullHead = fullHead < QUEUE_LAST ? fullHead + 1 : 0;
                    curBlock = 0;
                }
            }
        }
        if (fullHead == fullTail)
        {
          if (closeFile)
          {
              break;
          }
        }
        else if (!sd.card()->isBusy())
        {
            block_t* pBlock = fullQueue[fullTail];
            fullTail = fullTail < QUEUE_LAST ? fullTail + 1 : 0;
      
            uint32_t usec = micros();
            if (!sd.card()->writeData((uint8_t*)pBlock))
            {
                error("write data failed");
            }
            usec = micros() - usec;
            if (usec > maxLatency)
            {
                maxLatency = usec;
            }
            emptyStack[emptyTop++] = pBlock;
            bn++;
            if (bn == FILE_BLOCK_COUNT)
            {
                break;
            }
        }
    }
    if (!sd.card()->writeStop())
    {
        error("writeStop failed");
    }
    if (bn != FILE_BLOCK_COUNT)
    {
        Serial.println(F("Truncating file"));
        if (!binFile.truncate(512L * bn))
        {
            error("Can't truncate file");
        }
    }
    Serial.println("Done");
}

void dumpData(String fname)
{
    block_t block;
    if (!binFile.isOpen())
    {
        Serial1.println("No Data");
        return;
    }
    binFile.rewind();
    while (binFile.read(&block , 512) == 512)
    {
        if (block.count == 0)
        {
            Serial1.println("Size: 0");
            break;
        }
        else
        {
            Serial1.print("Size: ");
            Serial1.println(block.count);
        }
    }
    Serial.println("END");
}

void setup(void)
{
    pinMode(ERROR_LED_PIN, OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(ERROR_LED_PIN, HIGH);
    digitalWrite(STATUS_LED_PIN, HIGH);
  
    Serial.begin(9600);
    
    Serial.println("Starting...");
    Serial1.begin(9600);
    while(!Serial1)
    {
        ;
    }
    rampSetup();
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(12)))
    {
        sd.initErrorPrint(&Serial1);
        fatalBlink();
    }
    digitalWrite(ERROR_LED_PIN, LOW);
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(STATUS_LED_PIN, HIGH);
}

void loop(void)
{
    if(Serial1.available())
    {
        String cmd = Serial1.readString();
        if(strcmp(cmd.c_str(), "START"))
        {
            while(!Serial1.available())
            {
                ;
            }
            String fname = Serial1.readString();
            Serial1.println("STARTING");
            logData(fname);
        }
        else if(strcmp(cmd.c_str(), "SEND"))
        {
            while(!Serial1.available())
            {
                ;
            }
            String fname = Serial1.readString();
            Serial1.println("SENDING");
            dumpData(fname);
        }
        else if(strcmp(cmd.c_str(), "VERSION"))
        {
            Serial1.println("4.0.1");
        }
        else if(strcmp(cmd.c_str(), "RESET"))
        {
            Serial1.println("RESETTING");
            if (!sd.wipe(&Serial1))
            {
                sd.errorHalt("Wipe failed.");
            }
            else
            {
                Serial1.println("Done");
            }
        }
        else if(strcmp(cmd.c_str(), "?"))
        {
            Serial1.println("-START");
            Serial1.println("-stop");
            Serial1.println("-SEND");
            Serial1.println("-VERSION");
            Serial1.println("-RESET");
            Serial1.println("-?");      
        }
        else
        {
            Serial1.print("Unknown command\"");
            Serial1.print(cmd);
            Serial1.println("\", try '?'");  
        }
    }
} 
