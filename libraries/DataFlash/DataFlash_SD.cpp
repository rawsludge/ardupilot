/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *
 */
#include <AP_HAL.h>
#include <MySD.h>
#include "DataFlash.h"
#include <stdlib.h>
#include <AP_Param.h>
#include <AP_Math.h>

extern const AP_HAL::HAL& hal;

/*
  try to take a semaphore safely from both in a timer and outside
 */

DataFlash_SD::DataFlash_SD() :
	_initialised(false)
{

}	    

// Public Methods //////////////////////////////////////////////////////////////
void DataFlash_SD::Init(const struct LogStructure *structure, uint8_t num_types)
{
    if( _initialised ) return ;
    if( !SD.begin() )
    {
        hal.console->println_P(PSTR("DataFlash_SD not initialized. error\n"));
        return;
    }
    _initialised = true;
}

// This function return 1 if Card is inserted on SD slot
bool DataFlash_SD::CardInserted()
{
    return _initialised;
}

// erase handling
bool DataFlash_SD::NeedErase(void)
{
    // we could add a format marker at the start of a file?
    return false;
}

void DataFlash_SD::EraseAll()
{

}

void DataFlash_SD::WriteBlock(const void *pBuffer, uint16_t size)
{
    if( !_initialised || !_currentFile ) return;
    
    if( !_currentFile.write(pBuffer, size))
        hal.console->println_P(PSTR("Write failed"));
    //_currentFile.flush();
}

uint16_t  DataFlash_SD::find_last_log()
{
	return _get_file_count();
}

void DataFlash_SD::get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page)
{

}

uint16_t DataFlash_SD::get_num_logs(void)
{
	return _get_file_count();
}

void DataFlash_SD::LogReadProcess(uint16_t log_num,
                                  uint16_t start_page, uint16_t end_page,
                                  void (*printMode)(AP_HAL::BetterStream *port, uint8_t mode),
                                  AP_HAL::BetterStream *port)
{

}

void DataFlash_SD::DumpPageInfo(AP_HAL::BetterStream *port)
{
    port->printf_P(PSTR("DataFlash: num_logs=%u\n"),
                   (unsigned)get_num_logs());
}

void DataFlash_SD::ShowDeviceInfo(AP_HAL::BetterStream *port)
{
    //port->printf_P(PSTR("DataFlash logs stored in %s\n"),
    //               _log_directory);
}

void DataFlash_SD::ListAvailableLogs(AP_HAL::BetterStream *port)
{
    uint16_t num_logs = get_num_logs();
    if (num_logs == 0) {
        port->printf_P(PSTR("\nNo logs\n\n"));
        return;
    }
    port->printf_P(PSTR("\n%u logs\n"), (unsigned)num_logs);
    
    File dir = SD.open("/");
    dir.rewindDirectory();
    num_logs = 0;
    while (true) {
        File entry =  dir.openNextFile();
        if( !entry )
            break;
        if( !entry.isDirectory() )
            port->printf_P(PSTR("%s\n"), entry.name());
        entry.close();
    }
    dir.close();
}

uint16_t DataFlash_SD::start_new_log(void)
{
    if( !_initialised) return 0;
    
    char buffer[13];
    memset(buffer, 13, 0);
    
    if( _currentFile )
        _currentFile.close();

    uint8_t log_num = _get_file_count();
    hal.util->snprintf(buffer, 12, "%08u.bin", (unsigned)log_num);
    buffer[12]='\0';
    
    _currentFile = SD.open(buffer, O_CREAT|O_WRITE|O_APPEND);
    if( !_currentFile )
        hal.console->print_P(PSTR("File open error\n"));
    
	return log_num;
}

void DataFlash_SD::ReadBlock(void *pkt, uint16_t size)
{
    if( !_initialised || !_currentFile ) return;
    
    if( !_currentFile.read(pkt, size) )
        hal.console->println_P(PSTR("Read failed"));
}

/*
    private members
 */

int8_t DataFlash_SD::_get_file_count()
{
    if( !_initialised) return 0;

    File dir = SD.open("/");
    dir.rewindDirectory();
    int8_t fileCount = 1;
    
    while(true)
    {
        File entry =  dir.openNextFile();
        if( !entry )
            break;
        if( !entry.isDirectory() ) fileCount++;
        entry.close();
    }
    dir.close();
        
    return fileCount;
}



