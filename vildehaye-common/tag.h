/*
 * tag.h
 *
 *      Author: stoledo
 */

#ifndef TAG_H_
#define TAG_H_

#ifdef USE_TAG

extern uint64_t tagId;
extern uint16_t tagPeriodMs;

extern int32_t  tag_setId(uint64_t id);
extern void atlas_setCode(uint8_t index, const uint8_t* p, uint16_t len);
extern void     tagTask_init();
extern void     schedule_init(const uint16_t* configurationData, uint16_t configurationDataLength);
extern void tag_wakeupTrigger();
extern void tag_gotoConfiguration(uint8_t c);
extern void tag_triggerWakeupSetup(const uint8_t* wtarray);
extern void tag_triggerNoWakeupSetup(const uint16_t* nwtarray);
extern void tag_triggerWakeup(uint8_t trigger);
extern void tag_setVoltageThreshold( uint32_t vth );
extern uint8_t tag_sessionSeqno(uint32_t seqno);

#endif // tag firmware

#endif /* TAG_H_ */

