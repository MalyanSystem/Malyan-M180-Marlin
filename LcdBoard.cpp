#include "LcdBoard.h"
#include "temperature.h"
#include "Marlin.h"
#include "cardreader.h"

#include <avr/eeprom.h>
#include <avr/wdt.h>

#include <stdlib.h>

int16_t jog_speed=250;

inline void put(char c) 
{
   while (!(UCSR3A & (1<<UDRE3))) {};
   UDR3 = c; 
}

void writeString(char message[]) {
    char* letter = message;
    while (*letter != 0) {
        put(*letter);
        letter++;
    }
}

void writeInt(uint16_t value, uint8_t digits) {
    uint32_t currentDigit, nextDigit, uvalue;
    
    switch (digits) {
        case 1:  currentDigit = 10;	     break;
        case 2:  currentDigit = 100;     break;
        case 3:  currentDigit = 1000;    break;
        case 4:  currentDigit = 10000;   break;
        case 5:  currentDigit = 100000;  break;
        case 6:  currentDigit = 1000000;  break;
        default: return;
    }
    
    uvalue = (uint32_t)value;
    for (uint8_t i = 0; i < digits; i++) {
        nextDigit = currentDigit / 10;
        put((uvalue % currentDigit) / nextDigit + '0');
        currentDigit = nextDigit;
    }
}

void writeInt32(uint32_t value, uint8_t digits) {
    uint32_t currentDigit, nextDigit, uvalue;
    
    switch (digits) {
        case 1:  currentDigit = 10;      break;
        case 2:  currentDigit = 100;     break;
        case 3:  currentDigit = 1000;    break;
        case 4:  currentDigit = 10000;   break;
        case 5:  currentDigit = 100000;  break;
        case 6:  currentDigit = 1000000;  break;
        case 7:  currentDigit = 10000000;  break;
        default: return;
    }
    
    uvalue = (uint32_t)value;
    for (uint8_t i = 0; i < digits; i++) {
        nextDigit = currentDigit / 10;
        put((uvalue % currentDigit) / nextDigit + '0');
        currentDigit = nextDigit;
    }
}

// Use double-speed mode for more accurate baud rate?
    #define UBRR0_VALUE 16 // 115200 baud
    #define UBRR1_VALUE 51 // 38400 baud
    #define UBRR3_VALUE 16 // 115200 baud
    #define UCSRA_VALUE(uart_) _BV(U2X##uart_)

    // Adapted from ancient arduino/wiring rabbit hole
    #define INIT_SERIAL(uart_) \
    { \
        UBRR##uart_##H = UBRR##uart_##_VALUE >> 8; \
        UBRR##uart_##L = UBRR##uart_##_VALUE & 0xff; \
        \
        /* set config for uart_ */ \
        UCSR##uart_##A = UCSRA_VALUE(uart_); \
        UCSR##uart_##B = _BV(RXEN##uart_) | _BV(TXEN##uart_); \
        UCSR##uart_##C = _BV(UCSZ##uart_##1)|_BV(UCSZ##uart_##0); \
    }
	
void malyan_lcd_init() {
	// Port Register
	PORTJ |= 1<<0;			// TXD logic high
	PORTJ &= ~(1<<1);			// RXD high impedance
	
	// Data Direction
	DDRJ &= ~(1<<1);			// RXD Input
	DDRJ |= 1<<0;				// TXD Output
	
	INIT_SERIAL(3);
    //building = false;

    buff_state=0;
    writeString((char *)"{SYS:STARTED}");
    writeString((char *)"{U:RG1R180180120P0L1S0D0O1E1H0C0X1Y1Z1A2B2N3M0}");
}


//extern uint8_t file_from_wifi;
//extern uint8_t countFiles(bool update);
//extern bool getFilename(uint8_t index, char buffer[], uint8_t buffer_size, uint8_t *buflen, bool *isdir);

uint8_t countFiles(bool update)
{
	uint16_t nr;
	nr = card.getnrfilenames();
	if (nr>200) nr=200;
	return nr;
}

void ListFile(uint8_t index, uint8_t nr)
{
	//card.getfilename(nr-1-index);
	card.getfilename(index);
	
	if (card.filenameIsDir) writeString((char *)"{DIR:");
	else writeString((char *)"{FILE:");
	if (card.longFilename[0]!=0) writeString(card.longFilename);
	else writeString(card.filename);
	put('}');
	
	//MYSERIAL.println(card.longFilename);
}

void StartBuild()
{
    writeString((char *)"{SYS:BUILD}");
}

void EndBuild()
{
	writeString((char *)"{SYS:ENDOFBUILD}");
}

void PrintingStatus()
{
    int16_t t;
    int32_t t32;
    
    writeString((char *)"{T0:");
    t=degHotend(0);
    if (t>999) t=999;
    writeInt(t,3);
    put('/');
    t=degTargetHotend(0);
    writeInt(t,3);
    put('}');
    
    writeString((char *)"{T1:");
    t=degHotend(1);
    if (t>999) t=999;
    writeInt(t,3);
    put('/');
    t=degTargetHotend(1);
    writeInt(t,3);
    put('}');
    
    writeString((char *)"{TP:");
    t=degBed();
    if (t>999) t=999;
    writeInt(t,3);
    put('/');
    t=degTargetBed();
    writeInt(t,3);
    put('}');
    
    writeString((char *)"{TQ:");
    t=card.percentDone();
    writeInt(t,3);
	
	if (card.sdprinting) put('P');
	else put('C');
    /*switch(host::getHostState())
    {
        case host::HOST_STATE_BUILDING_ONBOARD:
        case host::HOST_STATE_BUILDING:
        case host::HOST_STATE_BUILDING_FROM_SD:
            put('P');
            break;
            
        case host::HOST_STATE_HEAT_SHUTDOWN:
            put('S');
            break;
            
        default:
            put('C');
            break;
    }*/
    put('}');
    
    writeString((char *)"{TT:");
    if(starttime != 0) t32 = millis()/60000 - starttime/60000;
	else t32=0;
    writeInt32(t32,6);
    put('}');
    
    writeString((char *)"{TR:");
    //t32=command::estimatedTimeLeftInSeconds();
	t32=0;
    writeInt32(t32,6);
    put('}');
    
    writeString((char *)"{TF:");
    //t32=command::filamentUsed();
	t32=0;
    writeInt32(t32,6);
    put('}');
}

void process()
{
    switch (buff_obj[0]) {
        case 'V':
            writeString((char *)"{VER:008}");
            return;

        case 'S':
            if (buff_value[0]=='E') writeString((char *)"{SYS:echo}");
/*            else if (buff_value[0]=='H')
            {
                uint8_t i,itemCount;
                
                itemCount=countFiles(false);
                //if (file_from_wifi!=0)
                {
                    writeString((char *)"{WIFI:");
                    writeInt(file_from_wifi,3);
                    put('/');
                    writeInt(itemCount,3);
                    put('}');
                }
            }*/
            else if (buff_value[0]=='L')
            {
                uint8_t i;
                uint8_t itemCount;
                
                if ( card.isFileOpen() ) {
                    writeString((char *)"{SYS:BUSY}");
                    return;
                }
				
				//card.initsd();
				//card.getWorkDirName();

                itemCount = countFiles(true);
				
				if (itemCount==0)
				{
					card.initsd();
					card.setroot();
					itemCount = countFiles(true);
				}
				
                for (i=0;i<itemCount;i++)
                {
                    ListFile(i,itemCount);
                }

                if (itemCount==0)
                {
                    if (card.cardOK) i=101;
					else i=102;
                    writeString((char *)"{ERR:");
                    writeInt(i,3);
                    put('}');
                }
                else writeString((char *)"{SYS:OK}");
            }
            else if (buff_value[0]=='I')
            {
                int16_t t;
                                
				writeString((char *)"{T0:");
				t=degHotend(0);
				if (t>999) t=999;
				writeInt(t,3);
				put('/');
				t=degTargetHotend(0);
				writeInt(t,3);
				put('}');
				
				writeString((char *)"{T1:");
				t=degHotend(1);
				if (t>999) t=999;
				writeInt(t,3);
				put('/');
				t=degTargetHotend(1);
				writeInt(t,3);
				put('}');
				
				writeString((char *)"{TP:");
				t=degBed();
				if (t>999) t=999;
				writeInt(t,3);
				put('/');
				t=degTargetBed();
				writeInt(t,3);
				put('}');
            }
            else if (buff_value[0]=='F')
            {
                /*if (buff_value[1]=='X')
                {
                    eeprom::setEepromInt64(eeprom_offsets::FILAMENT_TRIP, eeprom::getEepromInt64(eeprom_offsets::FILAMENT_LIFETIME, 0));
                    eeprom::setEepromInt64(eeprom_offsets::FILAMENT_TRIP + sizeof(int64_t), eeprom::getEepromInt64(eeprom_offsets::FILAMENT_LIFETIME + sizeof(int64_t), 0));
                }*/
                
                writeString((char *)"{TU:");
                
                uint16_t total_hours;
                uint8_t total_minutes;
                //eeprom::getBuildTime(&total_hours, &total_minutes);
				total_hours=0;
				total_minutes=0;
                writeInt(total_hours,5);
                put('.');
                writeInt(total_minutes,2);
                put('/');
                
                uint8_t build_hours;
                uint8_t build_minutes;
                //host::getPrintTime(build_hours, build_minutes);
				build_hours=0;
				build_minutes=0;
                writeInt(build_hours,3);
                put('.');
                writeInt(build_minutes,2);
                put('/');
                
                uint32_t filamentUsedA,filamentUsedB,filamentUsed;
                char str[11];
                //filamentUsedA=stepperAxisStepsToMM(eeprom::getEepromInt64(eeprom_offsets::FILAMENT_LIFETIME, 0),                  A_AXIS);
                //filamentUsedB=stepperAxisStepsToMM(eeprom::getEepromInt64(eeprom_offsets::FILAMENT_LIFETIME + sizeof(int64_t),0), B_AXIS);
                //filamentUsed=filamentUsedA+filamentUsedB;
				filamentUsed=0;
                itoa(filamentUsed,str,10);
                writeString((char *)str);
                put('/');
                
                //filamentUsedA -= stepperAxisStepsToMM(eeprom::getEepromInt64(eeprom_offsets::FILAMENT_TRIP, 0),                  A_AXIS);
                //filamentUsedB -= stepperAxisStepsToMM(eeprom::getEepromInt64(eeprom_offsets::FILAMENT_TRIP + sizeof(int64_t),0), B_AXIS);
                //filamentUsed=filamentUsedA+filamentUsedB;
				filamentUsed=0;
                itoa(filamentUsed,str,10);
                writeString((char *)str);
                put('}');
            }
            else if (buff_value[0]=='R' && 
                buff_value[1]=='E' &&
                buff_value[2]=='S' &&
                buff_value[3]=='E' &&
                buff_value[4]=='T')
            {
                //Motherboard::getBoard().reset(true);
				
            }
/*
            else if (buff_value[0]=='S')
            {
                writeString((char *)"{SYS:P");
                uint8_t i = command::pauseState();
                writeInt(i,3);
                put('/');
                put('H');
                i=host::getHostState();
                writeInt(i,3);
                put('}');
            }
            break;
            */
		
        case 'C':
            if (buff_value[0]=='P')
            {
                uint16_t t;
                
                t=atoi((const char*)buff_value+1);
                
                if (t<0 || t>150) return;
                setTargetBed(t);
            }
            else if (buff_value[0]=='T')
            {
                int16_t t;
                
                t=atoi((const char*)buff_value+2);
                if (t<0 || t>280) return;
                
                if (buff_value[1] == '0')
                {
                    setTargetHotend(t,0);
                }
                else
                {
                    setTargetHotend(t,1);
                }
            }
            else if (buff_value[0]=='S')
            {
                int16_t t;
                uint8_t i;
                
                t=atoi((const char*)buff_value+1);
                
                if (t<1) t=1;
                else if (t>50) t=50;
                
                feedmultiply=t*10;
            }
            break;
            
        case 'P':
            uint8_t i;
            if (buff_value[0]=='H')
            {
            	enquecommand_P(PSTR("G28"));
            }
            else if (buff_value[0]=='C')
            {
                //host::startOnboardBuild(utility::TOOLHEAD_CALIBRATE);
            }
            else if (buff_value[0]=='X')
            {
				extern bool cancel_heatup;
                writeString((char *)"{SYS:CANCELING}");
				//card.pauseSDPrint();
				//disable_heater();
				card.sdprinting = false;
				card.closefile();
				quickStop();
				if(SD_FINISHED_STEPPERRELEASE)
				{
					enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
				}
				autotempShutdown();
				cancel_heatup = true;
				writeString((char *)"{SYS:STARTED}");
				writeString((char *)"{U:RG1R180180120P0L1S0D0O1E1H0C0X1Y1Z1A2B2N3M0}");
				
            }
            else if (buff_value[0]=='P')
            {
                writeString((char *)"{SYS:PAUSE}");
                card.pauseSDPrint();
                writeString((char *)"{SYS:PAUSED}");
            }
            else if (buff_value[0]=='R')
            {
                writeString((char *)"{SYS:RESUME}");
                card.startFileprint();
                writeString((char *)"{SYS:RESUMED}");
            }
            else if (buff_value[0]=='Z')
            {
                /*i=(buff_value[1]-'0')*100 + (buff_value[2]-'0')*10 + (buff_value[3]-'0');
                float pauseAtZPos = i;
                command::pauseAtZPos(stepperAxisMMToSteps(pauseAtZPos, Z_AXIS));*/
            }
            else
            {
                i=(buff_value[0]-'0')*100 + (buff_value[1]-'0')*10 + (buff_value[2]-'0');
                card.getfilename(i);
				if (card.filenameIsDir)
				{
					writeString((char *)"{SYS:DIR}");
					card.chdir(card.filename);
				}
				else 
				{
					char cmd[30];
					char* c;
					
					writeString((char *)"{PRINTFILE:");
					if (card.longFilename[0]!=0) writeString(card.longFilename);
					else writeString(card.filename);
					put('}');
					
					sprintf_P(cmd, PSTR("M23 %s"), card.filename);
					for(c = &cmd[4]; *c; c++)
					*c = tolower(*c);
					enquecommand(cmd);
					enquecommand_P(PSTR("M24"));
				}
            }
            break;

        case 'B':
            PrintingStatus();
            break;

/*
        case 'J':
            switch (buff_value[0])
            {
                case 'S':
                    BOARD_STATUS_SET(Motherboard::STATUS_MANUAL_MODE);
                    jog_speed=atoi((const char*)buff_value+1);
                    break;
                    
                case 'E':
                    steppers::enableAxes(0xff, false);
                    BOARD_STATUS_CLEAR(Motherboard::STATUS_MANUAL_MODE);
                    break;
                    
                case 'X':
                case 'Y':
                case 'Z':
                case 'A':
                case 'B':
                    steppers::abort();
                    uint8_t dummy;
                    Point position = steppers::getStepperPosition(&dummy);
                    
                    int32_t t;
                    t=atoi((const char*)buff_value+1);

                    if (buff_value[0]<='B') position[buff_value[0]-'A'+3] += (t<<4);
                    else position[buff_value[0]-'X'] += (t<<4);
                    
                    steppers::setTargetNew(position, jog_speed, 0, 0);
                    break;
            }
            break;
*/
/*
        case 'H':
        	if (buff_value[0]=='R')
        	{
        		extern uint32_t homePosition[PROFILES_HOME_POSITIONS_STORED];

        		writeString((char *)"{H:R");
        		eeprom_read_block(homePosition, (void *)eeprom_offsets::AXIS_HOME_POSITIONS_STEPS, PROFILES_HOME_POSITIONS_STORED * sizeof(uint32_t));
        		writeInt(homePosition[0],5);
        		put('/');
        		writeInt(homePosition[1],5);
        		put('/');
        		writeInt(homePosition[2],5);
        		put('/');
        		writeInt((int32_t)(eeprom::getEeprom32(eeprom_offsets::TOOLHEAD_OFFSET_SETTINGS, 0)),5);
        		put('/');
        		writeInt((int32_t)(eeprom::getEeprom32(eeprom_offsets::TOOLHEAD_OFFSET_SETTINGS + sizeof(int32_t), 0)),5);
        		put('}');
        	}
        	else if (buff_value[0]=='W')
        	{
        		extern uint32_t homePosition[PROFILES_HOME_POSITIONS_STORED];
        		int32_t offset[2],t;
        		uint8_t axis;
        		axis=buff_value[1]-'X';
        		if (axis>=0 && axis<=2)
        		{
        			homePosition[axis]=atoi((const char*)buff_value+2);
        			cli();
					eeprom_write_block((void *)&homePosition[axis],
					   (void*)(eeprom_offsets::AXIS_HOME_POSITIONS_STEPS + sizeof(uint32_t) * axis) ,
					   sizeof(uint32_t));
					sei();
        		}

        		axis=buff_value[1]-'x';
        		if (axis>=0 && axis<=1)
        		{
        			t=atoi((const char*)buff_value+2);

                    int32_t offset[2];
                    bool    smallOffsets;

                    offset[0]  = (int32_t)(eeprom::getEeprom32(eeprom_offsets::TOOLHEAD_OFFSET_SETTINGS, 0));
                    offset[1]  = (int32_t)(eeprom::getEeprom32(eeprom_offsets::TOOLHEAD_OFFSET_SETTINGS + sizeof(int32_t), 0));
                    smallOffsets = abs(offset[0]) < ((int32_t)stepperAxisStepsPerMM(0) << 2);

                    int32_t delta = stepperAxisMMToSteps((float)(t - 7) * 0.1f, axis);
                    if ( !smallOffsets ) delta = -delta;

                    int32_t new_offset = offset[axis] + delta;
                    eeprom_write_block((uint8_t *)&new_offset,
                           (uint8_t *)eeprom_offsets::TOOLHEAD_OFFSET_SETTINGS + axis * sizeof(int32_t),
                           sizeof(int32_t));
        		}
        	}
        	break;
*/
/*
        case 'U':
            if (buff_value[0]=='R')
            {
            	int temp;

                writeString((char *)"{U:RG");
                if (eeprom::getEeprom8(eeprom_offsets::OVERRIDE_GCODE_TEMP, 0) != 0) put('1');
                else put('0');

                put('R');
                temp=eeprom::getEeprom16(eeprom_offsets::PREHEAT_SETTINGS + preheat_eeprom_offsets::PREHEAT_LEFT_OFFSET, DEFAULT_PREHEAT_TEMP);
                writeInt(temp,3);
                //put('/');
                temp=eeprom::getEeprom16(eeprom_offsets::PREHEAT_SETTINGS + preheat_eeprom_offsets::PREHEAT_RIGHT_OFFSET, DEFAULT_PREHEAT_TEMP);
                writeInt(temp,3);
                //put('/');
                temp=eeprom::getEeprom16(eeprom_offsets::PREHEAT_SETTINGS + preheat_eeprom_offsets::PREHEAT_PLATFORM_OFFSET, DEFAULT_PREHEAT_TEMP);
                writeInt(temp,3);

                put('P');
                if (eeprom::hasHBP() != 0) put('1');
                else put('0');
                
                put('L');
                if (eeprom::getEeprom8(eeprom_offsets::ACCELERATION_SETTINGS + acceleration_eeprom_offsets::ACCELERATION_ACTIVE, 0x01) != 0) put('1');
                else put('0');
                
                put('S');
                if (eeprom::getEeprom8(eeprom_offsets::COOL_PLAT, 0) != 0) put('1');
                else put('0');
                
                put('D');
                if (eeprom::getEeprom8(eeprom_offsets::DITTO_PRINT_ENABLED, 0) != 0) put('1');
                else put('0');
                
                put('O');
                if (eeprom::getEeprom8(eeprom_offsets::TOOLHEAD_OFFSET_SYSTEM,
                                       DEFAULT_TOOLHEAD_OFFSET_SYSTEM) != 0) put('1');
                else put('0');
                
                put('E');
                if (eeprom::getEeprom8(eeprom_offsets::EXTRUDER_HOLD,
                                       DEFAULT_EXTRUDER_HOLD) != 0) put('1');
                else put('0');
                
                put('H');
                if (eeprom::getEeprom8(eeprom_offsets::HEAT_DURING_PAUSE, DEFAULT_HEAT_DURING_PAUSE) != 0) put('1');
                else put('0');
                
                put('C');
                if (eeprom::getEeprom8(eeprom_offsets::SD_USE_CRC, DEFAULT_SD_USE_CRC) != 0) put('1');
                else put('0');
                
                put('X');
                put ('0' + eeprom::getEeprom8(eeprom_offsets::STEPPER_X_CURRENT, 0));
                
                put('Y');
                put ('0' + eeprom::getEeprom8(eeprom_offsets::STEPPER_Y_CURRENT, 0));
                
                put('Z');
                put ('0' + eeprom::getEeprom8(eeprom_offsets::STEPPER_Z_CURRENT, 0));
                
                put('A');
                put ('0' + eeprom::getEeprom8(eeprom_offsets::STEPPER_A_CURRENT, 0));
                
                put('B');
                put ('0' + eeprom::getEeprom8(eeprom_offsets::STEPPER_B_CURRENT, 0));

                put('N');
                put ('0' + eeprom::getEeprom8(eeprom_offsets::LANGUAGE, 0));

                put('M');
                put ('0' + eeprom::getEeprom8(eeprom_offsets::WIFI_SD, 0));
                
                put('}');
            }
            else if (buff_value[0]=='W')
            {
                uint8_t *c;
                uint8_t cmd=0;
                
                c=buff_value;
                while (*++c!=0)
                {
                    if (*c<='9' && *c>='0')
                    {
                        uint8_t value;
                        value = *c - '0';
                        
                        switch (cmd)
                        {
                            case 'G':
                                eeprom_write_byte((uint8_t *)eeprom_offsets::OVERRIDE_GCODE_TEMP,value);
                                break;
                                
                            case 'P':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::HBP_PRESENT, value);
                                break;
                                
                            case 'L':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::ACCELERATION_SETTINGS +
                                                  acceleration_eeprom_offsets::ACCELERATION_ACTIVE,
                                                  value);
                                break;
                                
                            case 'S':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::COOL_PLAT, value);
                                break;
                                
                            case 'D':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::DITTO_PRINT_ENABLED, value);
                                break;
                                
                            case 'O':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::TOOLHEAD_OFFSET_SYSTEM, value);
                                break;
                                
                            case 'E':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::EXTRUDER_HOLD, value);
                                break;
                                
                            case 'H':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::HEAT_DURING_PAUSE, value);
                                break;
                                
                            case 'C':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::SD_USE_CRC, value);
                                break;
                                
                            case 'T':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::PSTOP_ENABLE, value);
                                break;
                                
                            case 'X':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::STEPPER_X_CURRENT, value);
                                break;
                                
                            case 'Y':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::STEPPER_Y_CURRENT, value);
                                break;
                                
                            case 'Z':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::STEPPER_Z_CURRENT, value);
                                break;
                                
                            case 'A':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::STEPPER_A_CURRENT, value);
                                break;
                                
                            case 'B':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::STEPPER_B_CURRENT, value);
                                break;

                            case 'N':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::LANGUAGE, value);
                                break;

                            case 'M':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::WIFI_SD, value);
                                break;
                                
                            default:
                                break;
                        }
                    }
                    else if (*c<='Z' && *c>='A') cmd = *c;
                }
            }
            else if (buff_value[0] == 'U' &&
                    buff_value[1] == 'P' &&
                    buff_value[2] == 'D' &&
                    buff_value[3] == 'A' &&
                    buff_value[4] == 'T' &&
                    buff_value[5] == 'E')
            {
                char r;
                cli();
                wdt_disable();

                while (1)
                {
                    if (UCSR0A & (1<<RXC0))
                    {
                        r = UDR0;
                        UDR3 = r;
                    }

                    if (UCSR3A & (1<<RXC3))
                    {
                        r = UDR3;
                        UDR0 = r;
                    }
                }
            }
            else {
                if (buff_value[0]=='E' && buff_value[1]=='R' && buff_value[2]=='A' && buff_value[3]=='S' && buff_value[4]=='E')
                {
                    eeprom::factoryResetEEPROM();
                    Motherboard::getBoard().reset(true);
                }
                
                else if (buff_value[0]=='F' && buff_value[1]=='U' && buff_value[2]=='L' && buff_value[3]=='L' && buff_value[4]=='E' && buff_value[5]=='R' && buff_value[6]=='A' && buff_value[7]=='S' && buff_value[8]=='E')
                {
                    eeprom::erase();
                    host::stopBuildNow();
                }
            }
            break;
*/
        default:
            break;
    }
}

/*
 {FN:LIST},{}
 */
void malyan_lcd_update() {
    uint8_t c;
    while (UCSR3A & (1<<RXC3))
	{
        c = UDR3;
        put(c);
		switch (c)
		{
			case '{': if (buff_state==0)
                      {
                          buff_state=1;
                          buff_ptr=0;
                      }
                      break;
                
			case ':': if (buff_state==1)
                      {
                          buff_obj[buff_ptr++]=0;
                          buff_state=2;
                          buff_ptr=0;
                      }
                      break;
                
			case ',': if (buff_state==2)
                      {
                          buff_value[buff_ptr++]=0;
                          process();
                          buff_state=1;
                          buff_ptr=0;
                      }
                      else if (buff_state==1)
                      {
                          put('?');
                          buff_state=0;
                          buff_ptr=0;
                      }
                      break;
			
            case '}': if (buff_state==2)
                      {
                          buff_value[buff_ptr++]=0;
                          process();
                          buff_state=0;
                          buff_ptr=0;
                      }
                      else if (buff_state==1)
                      {
                          put('?');
                          buff_state=0;
                          buff_ptr=0;
                      }
                      break;

            default:  if (c>=0x21 && c<=0x7E)
                      {
                          if (buff_state==1) buff_obj[buff_ptr++]=c;
                          else if (buff_state==2) buff_value[buff_ptr++]=c;
                      }
                      else
                      {
                          put('?');
                          buff_state=0;
                          buff_ptr=0;
                      }
		}
	}
}

bool lcd_onboard_build = false;

