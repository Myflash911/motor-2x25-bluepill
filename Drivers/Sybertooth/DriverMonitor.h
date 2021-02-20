#ifndef __DRIVER_MONITOR
#define __DRIVER_MONITOR

#include <inttypes.h>
#include "XPIDController.h"
#include "SabertoothSimplified.h"
#include "MotorUnit.h"
#include "usart.h"

#define MAX_COMMAND_LENGTH 	255
#define ERR_LONG_COMMAND 	1


typedef enum __commandType {
	None,
	Basic,
	Extended
} commandType;

typedef struct __command_line
{
	char command_str[MAX_COMMAND_LENGTH+1];
	char ready;
	commandType type;
	int8_t cursor;
} command_line_t;



typedef struct __FLASH_sector {
	uint8_t data[1024-13];
	int8_t   threshold;
	uint32_t limit_gap;
	uint32_t NWrite;
	uint32_t CheckSum;
} FLASH_sector;  // size 1024 B


union NVRAM {
	PID_settings_t settings[2];
	FLASH_sector sector;
	uint32_t data32[256];
};



class DriverMonitor
{
private:
	command_line_t command;
	/**
	 * pointer to array for printout text messages. ~ 256B
	 */
	char* _output;

	/**
	 * calibration flag for preconfig
	 */
//	uint8_t _calibrationMode;

	/**
	 * disables basic comands execution and pid-commands for rotation.
	 * values:
	 * 0 - PID enabled - set after success calibration. Can be set to 0 by 'forceon' command
	 * 1 - PID disabled - default value. Can be set to 1 by 'forceoff' command
	 */
//	uint8_t _disabled;

	/**
	 * switcher for enable/disable pid-calc and pid-roll by basic commands.
	 */
//	uint8_t _pause;

//	uint16_t _last_hited_lmtsw[4];

	/**
	 * temp settings for operations.
	 */
	PID_settings_t _tmp_stng;

	/**
	 * block of mem to save/load data.
	 */
	NVRAM _DevNVRAM;

	SabertoothSimplified *_ST;


public:
	XPIDController *pPID_L;
	XPIDController *pPID_R;

	MotorUnit *pLeftMotorUnit;
	MotorUnit *pRightMotorUnit;
	/**
	 * space on both side of scale form limit switch to allowed position.
	 */
//	uint32_t limit_gap;

//	uint16_t target_L, target_R;

public:
	DriverMonitor();
	virtual ~DriverMonitor();

public:
	/**
	 * Executes Extended command
	 * SINTAX:
	 * 'get A' or 'get B'
	 * 'set A Kp 1.0 Ki 1.2e-6 Kd 2e-2 dz 5' or 'set B hb 2'
	 * List of parameters:
	 * Kd, Ki, Kp, Ka, dz
	 * @param cmd
	 * @return
	 */
	const char* 	ExecuteEx(char* cmd);

	/**
	 *
	 * @param cmd :
	 * LxxC | RxxC -- set L|R target as xx - binary uint16_t.
	 * E..C -- stop all motors
	 * @return
	 */
	void		 	ExecuteBasic(char* cmd);

	/**
	 * Load saved to flash setting
	 * @return
	 * 0 - succesfully loaded
	 * 1 - failed
	 */
	int  	Load(void);

	/**
	 * Sets _mL and _mR to default values.
	 */
	void 	Set_PID_default(void);

	/**
	 *
	 * @return
	 * 0 - succesfully calibrated
	 * x - error code
	 */
//	uint8_t Calibration(void);

	/**
	 * @descr: Инициализирует монитор для работы с моторами через ПИД-регуляторы.
	 * @param
	 * TxBuffer - Буфер для отправки отладоных сообщений
	 * ST - Указатель на готовый класс работы с двигателем Sybertooth.
	 */
	void Start(char* TxBuffer, SabertoothSimplified *pST);

//	void HitLimitSwitch(uint16_t lim_sw_pin);

//	void CalcPID(void);

	void printStatus(void);

//	void TestBlock(uint8_t motor, XPIDController* PID, uint16_t BWD_Pin, uint16_t FWD_Pin);


	void Parse(void);
	void Execute(void);

private:

	void _get_settings(void);
	uint8_t  _set_settings(void);
	void _parse_PID_param();
	void _set_PID_value(char* param);


	/**
	 *
	 * @return
	 * 0 - success
	 * 1 - error
	 */
	uint8_t _save(void);

	void _rotate(void);
	void _requestPIDModeOn(void);
};

uint32_t calc_flash_checksum(uint32_t* _data);

#endif
