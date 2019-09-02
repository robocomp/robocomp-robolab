#include <vs_can_api.h>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>

#include <stdint.h>

//DEVICE MODES
#define MODE_OFF 0x00000000
#define MODE_CURRENT 0x00000002
#define MODE_VEL 0x00000003
#define MODE_SUBVEL 0x00000005
#define MODE_POS 0x00000007


#define DEFAULT_DUNKERMOTOR_MAXPOS INT_MAX
#define DEFAULT_DUNKERMOTOR_MINPOS INT_MIN

//~ #define MAX_DUNKERMOTOR_STEPS 286720

#define MAX_READ_RETRIES 200


void Dunker_printMessageData(VSCAN_MSG msg);

uint8_t Dunker_getCommandSpecifier(bool _write, bool _expedited, bool _sizeSpecified);

uint8_t Dunker_getDefaultReadCommandSpecifier(void);

int Dunker_getReadedData(VSCAN_MSG msg);

uint8_t Dunker_getDefaultWriteCommandSpecifier(void);

VSCAN_MSG Dunker_getMessageData(uint8_t nodeId, uint8_t CommandSpecifier, uint16_t obj_id, uint8_t obj_subid, uint32_t obj_data=0);

int Dunker_writeWaitReadMessage(VSCAN_HANDLE Handle, VSCAN_MSG* msg);

int Dunker_multiWriteWaitReadMessage(VSCAN_HANDLE Handle, VSCAN_MSG* msgs, int msg_count);

//--------------------------------------

int Dunker_setBaudRate(VSCAN_HANDLE, int baudRate);

int Dunker_disablePower(VSCAN_HANDLE Handle, uint8_t NodeId);

int Dunker_enablePower(VSCAN_HANDLE Handle, uint8_t NodeId);

int Dunker_openBreak(VSCAN_HANDLE Handle, uint8_t NodeId);

int Dunker_clearError(VSCAN_HANDLE Handle, uint8_t NodeId);

int Dunker_setOperationMode(VSCAN_HANDLE Handle, uint8_t NodeId, uint32_t mode);
int Dunker_getOperationMode(VSCAN_HANDLE Handle, uint8_t NodeId, int* mode);


int Dunker_setVel(VSCAN_HANDLE Handle, uint8_t NodeId, int velocity);

int Dunker_getVel(VSCAN_HANDLE Handle, uint8_t NodeId, int* vel);

//~ int Dunker_setDinamicLimitations(VSCAN_HANDLE Handle, uint8_t NodeId);

int Dunker_setPositiveCurrentLimit(VSCAN_HANDLE Handle, uint8_t NodeId, int limit);
int Dunker_setNegativeCurrentLimit(VSCAN_HANDLE Handle, uint8_t NodeId, int limit);
int Dunker_setDynCurrentLimitOff(VSCAN_HANDLE Handle, uint8_t NodeId);
int Dunker_setDynCurrentLimitOn(VSCAN_HANDLE Handle, uint8_t NodeId);
int Dunker_setDynCurrentLimitPeak(VSCAN_HANDLE Handle, uint8_t NodeId, int limit);
int Dunker_setDynCurrentLimitContinous(VSCAN_HANDLE Handle, uint8_t NodeId, int limit);
int Dunker_setDynCurrentLimitTime(VSCAN_HANDLE Handle, uint8_t NodeId, int time);
int Dunker_syncPositiveCurrentLimit(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* limit);
int Dunker_syncNegativeCurrentLimit(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* limit);
int Dunker_syncDynCurrentLimitOff(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds);
int Dunker_syncDynCurrentLimitOn(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds);
int Dunker_syncDynCurrentLimitPeak(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* limit);
int Dunker_syncDynCurrentLimitContinous(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* limit);
int Dunker_syncDynCurrentLimitTime(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* limit);
int Dunker_syncSetEncoderRes(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* resol);




int Dunker_configureErrorsAndFeedback(VSCAN_HANDLE Handle, uint8_t NodeId);

int Dunker_resetModule(VSCAN_HANDLE Handle, uint8_t NodeId);

//set the maximum deviation between command and actual position
int Dunker_setMaxPosFollowingError(VSCAN_HANDLE Handle, uint8_t NodeId, uint32_t max_err);


int Dunker_setActualPosition(VSCAN_HANDLE Handle, uint8_t NodeId, uint32_t pos);

int Dunker_clearActualPosition(VSCAN_HANDLE Handle, uint8_t NodeId);


int Dunker_moveToAbsolutePosition(VSCAN_HANDLE Handle, uint8_t NodeId, int pos);

int Dunker_moveToRelativePosition(VSCAN_HANDLE Handle, uint8_t NodeId, uint32_t pos);


int Dunker_setAcceleration(VSCAN_HANDLE Handle, uint8_t NodeId, uint32_t deltaV, uint32_t deltaT);
 
int Dunker_setDecceleration(VSCAN_HANDLE Handle, uint8_t NodeId, uint32_t deltaV, uint32_t deltaT);

int Dunker_getAcceleration(VSCAN_HANDLE Handle, uint8_t NodeId, int* deltaV, int* deltaT);
 
int Dunker_getDecceleration(VSCAN_HANDLE Handle, uint8_t NodeId, int* deltaV, int* deltaT);


int Dunker_getPos(VSCAN_HANDLE Handle, uint8_t NodeId, int* pos);

int Dunker_getActualCurrent(VSCAN_HANDLE Handle, uint8_t NodeId, int* curr);

//Decimas de grado
int Dunker_getMotorTemperature(VSCAN_HANDLE Handle, uint8_t NodeId, int* temp);

//Decimas de grado
int Dunker_getPowerStageTemperature(VSCAN_HANDLE Handle, uint8_t NodeId, int* stemp);

int Dunker_changeNodeId(VSCAN_HANDLE Handle, uint8_t NodeId, uint8_t newNodeId);

int Dunker_setMaxPos(VSCAN_HANDLE Handle, uint8_t NodeId, int maxPos);

int Dunker_getMaxPos(VSCAN_HANDLE Handle, uint8_t NodeId, int* pos);

int Dunker_setMinPos(VSCAN_HANDLE Handle, uint8_t NodeId, int minPos);

int Dunker_getMinPos(VSCAN_HANDLE Handle, uint8_t NodeId, int* pos);

int Dunker_setMaxVelPositive(VSCAN_HANDLE Handle, uint8_t NodeId, int vel);

int Dunker_setMaxVelNegative(VSCAN_HANDLE Handle, uint8_t NodeId, int vel);

int Dunker_getMaxVelPositive(VSCAN_HANDLE Handle, uint8_t NodeId, int* vel);

int Dunker_getMaxVelNegative(VSCAN_HANDLE Handle, uint8_t NodeId, int* vel);

int Dunker_getMaxMotorVoltage(VSCAN_HANDLE Handle, uint8_t NodeId, int* vol);

int Dunker_getPower(VSCAN_HANDLE Handle, uint8_t NodeId, int* pow);

int Dunker_syncMoveToAbsolutePosition(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* pos);

int Dunker_syncMoveToRelativePosition(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* pos);

int Dunker_syncDisablePower(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds);

int Dunker_syncEnablePower(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds);

int Dunker_syncOpenBreak(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds);

int Dunker_syncClearError(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds);

int Dunker_syncSetOperationMode(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, uint32_t mode);

int Dunker_syncSetVel(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* velocity);

//~ int Dunker_syncSetDinamicLimitations(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds);

int Dunker_syncConfigureErrorsAndFeedback(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds);

int Dunker_syncSetCurrentLimitPos(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* limits);

int Dunker_syncSetCurrentLimitNeg(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* limits);
//POSITIVE AND NEGATIVE
int Dunker_syncSetCurrentLimit(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* limits);

int Dunker_syncSetMaxPosFollowingError(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, uint32_t max_err);

int Dunker_syncSetActualPosition(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, uint32_t* pos);

int Dunker_syncClearActualPosition(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds);

int Dunker_syncSetAcceleration(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* deltaV, int* deltaT);
 
int Dunker_syncSetDecceleration(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* deltaV, int* deltaT);

int Dunker_syncResetModule(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds);

int Dunker_syncSetMaxPos(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* maxPos);

int Dunker_syncSetMinPos(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* minPos);

int Dunker_syncSetMaxVelPositive(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* vels);

int Dunker_syncSetMaxVelNegative(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* vels);

int Dunker_getStatusWord(VSCAN_HANDLE Handle, uint8_t NodeId, int* status_word);

int Dunker_getErrorRegister(VSCAN_HANDLE Handle, uint8_t NodeId, int* error_register);

void Dunker_printStatusWordInfo(uint16_t status_word);

bool Dunker_errorFlag(uint32_t status_word);

int Dunker_setRampType(VSCAN_HANDLE Handle, uint8_t NodeId, uint32_t type);

int Dunker_syncSetRampType(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, uint32_t type);

int Dunker_setRampType(VSCAN_HANDLE Handle, uint8_t NodeId, uint32_t type);

int Dunker_syncSetRampType(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, uint32_t type);

int Dunker_getGearSaftRevolution(VSCAN_HANDLE Handle, uint8_t NodeId,  int* revol);

int Dunker_getMotorSaftRevolution(VSCAN_HANDLE Handle, uint8_t NodeId,  int* revol);

int Dunker_setGearSaftRevolution(VSCAN_HANDLE Handle, uint8_t NodeId,  int revol);

int Dunker_setMotorSaftRevolution(VSCAN_HANDLE Handle, uint8_t NodeId,  int revol);

int Dunker_syncGetPosition(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* positions);

int Dunker_syncGetVelocity(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* velocities);

int Dunker_syncGetStatusWord(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* status_words);

int Dunker_syncGetMotorVoltages(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* volts);

int Dunker_syncSetVelKps(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* kps);

int Dunker_syncSetVelKis(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* kis);

int Dunker_syncSetVelKds(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* kds);

int Dunker_syncGetVelKps(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* kps);

int Dunker_syncGetVelKis(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* kis);

int Dunker_syncGetVelKds(VSCAN_HANDLE Handle, int node_count, uint8_t* NodeIds, int* kds);

int Dunker_JoseMateos(VSCAN_HANDLE Handle);


