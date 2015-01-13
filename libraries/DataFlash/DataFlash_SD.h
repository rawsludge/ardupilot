/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* ************************************************************ */
/* DataFlash_SD Log library                                 */
/* ************************************************************ */
#ifndef __DATAFLASH_SD_H__
#define __DATAFLASH_SD_H__
#include <MySD.h>


class DataFlash_SD : public DataFlash_Class
{
private:
    //Methods
    volatile bool 	_initialised;
    uint16_t          _get_file_count();
    File            _currentFile;
    void            getFileName(uint16_t fileNum, char *buffer, int16_t size);

public:
	//initialize
	DataFlash_SD();
	
    void        Init(const struct LogStructure *structure, uint8_t num_types);
    bool        CardInserted();
    bool 		NeedErase(void);
    void 		EraseAll();
    void 		WriteBlock(const void *pBuffer, uint16_t size);
    uint16_t 	find_last_log();
    void 		get_log_boundaries(uint16_t, uint16_t&, uint16_t&);
    uint16_t 	get_num_logs();
    void        LogReadProcess(uint16_t log_num,
                   uint16_t start_page, uint16_t end_page,
                   void (*printMode)(AP_HAL::BetterStream *port, uint8_t mode),
                               AP_HAL::BetterStream *port);
    int16_t     get_log_data(uint16_t, uint16_t, uint32_t, uint16_t, uint8_t*);
    void        get_log_info(uint16_t, uint32_t&, uint32_t&);
    void 		DumpPageInfo(AP_HAL::BetterStream*);
    void 		ShowDeviceInfo(AP_HAL::BetterStream*);
	void 		ListAvailableLogs(AP_HAL::BetterStream*);
	uint16_t 	start_new_log();
	void 		ReadBlock(void*, uint16_t);

};

#endif
