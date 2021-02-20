/*
 * comand_parser.c
 *
 *  Created on: 23 авг. 2020 г.
 *      Author: Boris Pavlenko <borpavlenko@ispovedn1k.com>
 */


#include "DriverMonitor.h"
#include "minimath.h"
#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define MOTOR_L 				1U
#define MOTOR_R					2U

/// PID
#define PID_MS_INTERVAL			10U


/// FLASH SETTINGS
#define FLASH_SAVE_START_ADDR 	((uint32_t)0x0801FC00)
#define FLASH_SAVE_END_ADDR 	FLASH_SAVE_START_ADDR + FLASH_PAGE_SIZE
#define MAGIC_WORD				0x12345678

uint32_t tick, last_tick;


const char* LMTSWpinToName(uint16_t pin)
{
	if (pin == LMT1_BWD_Pin)
		return "LMT1_BWD";
	if (pin == LMT1_FWD_Pin)
		return "LMT1_FWD";
	if (pin == LMT2_BWD_Pin)
		return "LMT2_BWD";
	if (pin == LMT2_FWD_Pin)
		return "LMT2_FWD";
	return "UNKNOWN";
}


DriverMonitor::DriverMonitor() :
	_output(NULL)
{
	command.ready = 0;
	command.cursor = -1;
}


DriverMonitor::~DriverMonitor(void){}


void DriverMonitor::Start(char* TxBuffer, SabertoothSimplified *pST)
{
	_output = TxBuffer;
	_ST = pST;
}


const char* DriverMonitor::ExecuteEx(char *cmd)
{
	char* token = strtok(cmd, " ");

	if (strcmp(token, "get") == 0)
	{
		_get_settings();
	}
	else if (strcmp(token, "set") == 0)
	{
		_set_settings();
	}
	else if (strcmp(token, "save") == 0)
	{
		uint8_t save_result = _save();
		switch(save_result)
		{
		case 0:
			sprintf(_output, "\r\nSaved successfully.\r\n");
			break;
		case 1:
			sprintf(_output, "\r\nERROR! Failed to unlock flash for save.\r\n");
			break;
		case 2:
			sprintf(_output, "\r\nERROR! Failed to erase flash for save.\r\n");
		}
	}
	else if (strcmp(token, "load") == 0)
	{
		if(0 == Load())
		{
			sprintf(_output, "\r\nLoaded successfully\r\n");
		}
		else
		{
			sprintf(_output, "\r\nLoading failed\r\n");
		}
	}
	else if (strcmp(token, "stop") == 0)
	{
		_ST->stop();
		pLeftMotorUnit->Suspend();
		pRightMotorUnit->Suspend();
		sprintf(_output, "STOP.\r\n");
	}
	else if (strcmp(token, "rotate") == 0)
	{
		_rotate();
		sprintf(_output, "Executing rotation as command.\r\n");
	}
	else if (strcmp(token, "repeat") == 0)
	{
		return 0;
	}
	else if (strcmp(token, "calibrate") == 0)
	{
		pLeftMotorUnit->RequestCalibration();
		pRightMotorUnit->RequestCalibration();
		sprintf(_output, "Requested calibration\r\n");
	}
	else if (strcmp(token, "usepid") == 0)
	{
		_requestPIDModeOn();
	}
	else if (strcmp(token, "uselinear") == 0)
	{
		if(pLeftMotorUnit->UseLinear() && pRightMotorUnit->UseLinear())
		{
			sprintf(_output, "Switched to Linear mode\r\n");

		}
		else
		{
			sprintf(_output, "Linear mode rejected. Check Units' status.\r\n");
		}
	}
	else if (strcmp(token, "status") == 0)
	{
		printStatus();
	}
	else //if (strcmp(token, "help") == 0)
	{
		sprintf(_output, "\r\n SyberDrive %s \r\n\
Examples of commands:\r\n\
get <PID_L | PID_R | GAP>\r\n\
set PID_L Kp 1.0 Ki 1.2e-4 Kd 2.3 Ka 1.0 hb 1\r\n\
set GAP 50\r\n\
stop   - stop rotation.\r\n\
rotate <L|R> <speed>  - speed in [-127:127]. 0 for stop.\r\n\
repeat - repeats last contoller response.\r\n\
save   - save pid settings and gap to flash\r\n\
load   - load settings from flash.\r\n\
calibrate    - start to calibrate engines.\r\n\
usepid - \r\n\
uselinear - \r\n\
status <L|R> - get status.\r\n", VERSION);
	}

	return 0;
}


void DriverMonitor::_get_settings(void)
{
	char* token = strtok(NULL, " ");
	char* p = _output;

	if (strcmp(token, "pid_l") == 0)
	{
		p += sprintf(_output, "\r\n ===== PID L =====\r\n");
		pPID_L->sprint(p);
	}
	else if (strcmp(token, "pid_r") == 0)
	{
		p += sprintf(_output, "\r\n ===== PID R =====\r\n");
		pPID_R->sprint(p);
	}
	else if (strcmp(token, "gap") == 0)
	{
		p += sprintf(_output, "\r\n Left  GAP: %u \r\n", pLeftMotorUnit->limit_gap);
		p += sprintf(p, " Right GAP: %u \r\n", pRightMotorUnit->limit_gap);
	}
	else if (strcmp(token, "thr") == 0)
	{
		p += sprintf(_output, "\r\n thr: %u \r\n", _ST->threshold);
	}
}


uint8_t DriverMonitor::_set_settings(void)
{
	char* token = strtok(NULL, " ");
	char* p = _output;

	if (strcmp(token, "pid_l") == 0)
	{
		p += sprintf(_output, "\r\nUpdating config...\r\n ===== PID L =====\r\n");

		pPID_L->dumpPID(&_tmp_stng);
		_parse_PID_param();
		pPID_L->setPID(&_tmp_stng);
		pPID_L->sprint(p);
	}
	else if (strcmp(token, "pid_r") == 0)
	{
		p += sprintf(_output, "\r\nUpdating config...\r\n ===== PID R =====\r\n");
		pPID_R->dumpPID(&_tmp_stng);
		_parse_PID_param();
		pPID_R->setPID(&_tmp_stng);
		pPID_R->sprint(p);
	}
	else if (strcmp(token, "gap") == 0)
	{
		token = strtok(NULL, " ");
		pLeftMotorUnit->limit_gap = atoi(token);
		pRightMotorUnit->limit_gap = pLeftMotorUnit->limit_gap;
		sprintf(_output, "gap = %d\r\n", pLeftMotorUnit->limit_gap);
	}
	else if (strcmp(token, "thr") == 0)
	{
		token = strtok(NULL, " ");
		_ST->threshold = atoi(token);
		sprintf(_output, "thr = %d\r\n", _ST->threshold);
	}
	else
	{
		p += sprintf(_output, "\r\nERROR > Bad PID index. 'L' or 'R' expected, got: '%s'.\r\n", token);
		return 1;
	}

	return 0;
}

void DriverMonitor::_parse_PID_param()
{
	char* token = strtok(NULL, " ");
	while (token)
	{
		_set_PID_value(token);
		token = strtok(NULL, " ");
	}
}


void DriverMonitor::_set_PID_value(char* param)
{
	char * token = strtok(NULL, " ");
	if (!token) return;

	float val = atof(token);
	if (strcmp(param, "kp") == 0)
	{
		_tmp_stng.Kp = val;
		return;
	}
	if (strcmp(param, "ki") == 0)
	{
		_tmp_stng.Ki = val;
		return;
	}
	if (strcmp(param, "kd") == 0)
	{
		_tmp_stng.Kd = val;
		return;
	}
	if (strcmp(param, "ka") == 0)
	{
		_tmp_stng.Ka = val;
		return;
	}
	if (strcmp(param, "hb") == 0)
	{
		_tmp_stng.hitbox = (unsigned int) val;
		return;
	}
}


uint8_t DriverMonitor::_save(void)
{
	uint32_t l_Error = 0;
	uint32_t l_Address = FLASH_SAVE_START_ADDR;
	uint32_t l_Index = 0x00;

	FLASH_EraseInitTypeDef EraseInitStruct;

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_SAVE_START_ADDR;
	EraseInitStruct.NbPages = 1;

	pPID_L->dumpPID(&(_DevNVRAM.settings[0]));
	pPID_R->dumpPID(&(_DevNVRAM.settings[1]));

	_DevNVRAM.sector.NWrite += 1;
	_DevNVRAM.sector.limit_gap = pLeftMotorUnit->limit_gap;
	_DevNVRAM.sector.threshold = _ST->threshold;

	_DevNVRAM.sector.CheckSum = calc_flash_checksum(_DevNVRAM.data32);

	if (HAL_OK != HAL_FLASH_Unlock())
	{
		BLINK(); BLINK();
		return 1;
	}

	if (HAL_OK != HAL_FLASHEx_Erase(&EraseInitStruct, &l_Error))
	{
		BLINK(); BLINK(); BLINK();
		return 2;
	}

	while(l_Address < FLASH_SAVE_END_ADDR)
	{
		LED_ON();
		HAL_Delay(2);

		if (HAL_OK == HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, l_Address, _DevNVRAM.data32[l_Index]))
		{
			l_Index += 1;
			l_Address += 4;
		}

		LED_OFF();
		HAL_Delay(5);
	}

	HAL_FLASH_Lock();

	return 0;
}


int DriverMonitor::Load(void)
{
	uint32_t l_Address = FLASH_SAVE_START_ADDR;
	uint32_t l_Index = 0x00;

	while (l_Address < FLASH_SAVE_END_ADDR)
	{
		_DevNVRAM.data32[l_Index] = *(__IO uint32_t *)l_Address;
		l_Index += 1;
		l_Address += 4;
	}

	if (_DevNVRAM.sector.CheckSum == calc_flash_checksum(_DevNVRAM.data32))
	{
		pPID_L->setPID( &(_DevNVRAM.settings[0]));
		pPID_R->setPID( &(_DevNVRAM.settings[1]));
		//limit_gap = _DevNVRAM.sector.limit_gap;
		pLeftMotorUnit->limit_gap = _DevNVRAM.sector.limit_gap;
		pRightMotorUnit->limit_gap = _DevNVRAM.sector.limit_gap;
		_ST->threshold = _DevNVRAM.sector.threshold;
		// todo: Loading Minimal and Max speed settings for driver
		return 0;
	}

	return 1;
}


void DriverMonitor::printStatus(void)
{
	char* token = strtok(NULL, " ");
	char* p = _output;
	p += sprintf(p, " ==== Status ====\r\n");

	if (strcmp(token, "r") == 0)
	{
		p += pRightMotorUnit->GetStatus(p);
	}
	else
	{
		p += pLeftMotorUnit->GetStatus(p);
	}
}


/**
 * @descr basic command format: X---C
 */
void DriverMonitor::ExecuteBasic(char* cmd)
{
	if ('E' == cmd[0])
	{
		_ST->stop();
		pLeftMotorUnit->Stop();
		pRightMotorUnit->Stop();
		return;
	}

	uint16_t target = cmd[1] * 256 + cmd[2];

	if ('L' == cmd[0])
	{
		pLeftMotorUnit->SetTarget(target);
		pLeftMotorUnit->Resume();
		return;
	}

	if ('R' == cmd[0])
	{
		pRightMotorUnit->SetTarget(target);
		pRightMotorUnit->Resume();
		return;
	}
}


void DriverMonitor::Set_PID_default(void)
{
	pPID_L->setDefault();
	pPID_R->setDefault();
}


void DriverMonitor::_rotate(void)
{
	char* token = strtok(NULL, " ");
	if (0 == strcmp(token, "l"))
	{
		token = strtok(NULL, " ");
		pLeftMotorUnit->Rotate(atoi(token));
	}
	else if (0 == strcmp(token, "r"))
	{
		token = strtok(NULL, " ");
		pRightMotorUnit->Rotate(atoi(token));
	}
}


void DriverMonitor::Parse(void)
{
	char c;
	while (inq.parser_counter != inq.input_counter)
	{
		c = inq.str[inq.parser_counter];
		if (command.ready)
		{
			return;
		}
		if(command.cursor == -1)
		{
			// Ждём появления команды
			// Все команды начинаются с литеры 'X'
			if ('X' == c)
			{
				command.type = Basic;
				command.cursor = 0;
			}
			else if ('Y' == c)
			{
				command.type = Extended;
				command.cursor = 0;
			}
		}
		else // appending command char
		{
			command.command_str[command.cursor] = c;
			command.cursor++;

			if (command.type == Basic)
			{
				if (command.cursor == 4)
				{
					if (command.command_str[3] == 'C')
					{
						command.ready = 1;
						command.command_str[4] = '\0';
					}
					else
					{
						command.type = None;
						command.command_str[0] = 0;
					}
					command.cursor = -1;
				}
			} // endif (command.type == Basic)
			else if (command.type == Extended)
			{
				if (command.cursor == MAX_COMMAND_LENGTH)
				{
					command.cursor = -1;
					command.type = None;
				}
				else if (0x0A == c || 0x0D == c)
				{
					command.ready = 1;
					command.command_str[command.cursor-1] = 0;
					char *p = command.command_str;
					while (*p)
					{
						*p = lower(*p);
						p++;
					}
					command.cursor = -1;
				}
			} // endif (command.type == Extended)
		} // end else // appending command char
		inq.parser_counter++;
		if (inq.parser_counter == APP_RX_DATA_SIZE)
		{
			inq.parser_counter = 0;
		}
	} // end while
}


void DriverMonitor::Execute(void)
{
	_output[0] = 0;
	if (command.ready)
	{
		if (command.type == Extended)
		{
			ExecuteEx(command.command_str);
		}
		else if (command.type == Basic)
		{
			ExecuteBasic(command.command_str);
		}
		command.ready = 0;
		command.cursor = -1;
	}
	SAY(_output);
}


void DriverMonitor::_requestPIDModeOn(void)
{
	char *p = _output;
	p += sprintf(p, "Left ");
	if(pLeftMotorUnit->UsePID())
	{
		p += sprintf(p, "switched to PID mode\r\n");
	}
	else
	{
		p += sprintf(p, "PID mode rejected. Check Units' status.\r\n");
	}
	p += sprintf(p, "Right ");
	if(pRightMotorUnit->UsePID())
	{
		p += sprintf(p, "switched to PID mode\r\n");
	}
	else
	{
		p += sprintf(p, "PID mode rejected. Check Units' status.\r\n");
	}
}
