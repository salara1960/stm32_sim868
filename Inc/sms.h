#ifndef __SMS_H__
#define __SMS_H__

#include "def.h"

#ifdef SET_SMS

#include "libs.h"


#define SMS_BUF_LEN 512//600
#define maxSMSPart 8
#define MaxBodyLen 161//121//113

#define cod_PDU_len 159 //137
#define lenFrom 32
#define wait_sms_time 60 * 3
#define max_smsType 3

#pragma pack(push,1)
typedef struct s_udhi_t {
	uint8_t tp;//0-обычная смс, 1-часть длинной смс, 255-квитанция
	uint16_t num;//индекс (номер) смс
	uint8_t total;//количество частей
	uint8_t part;//номер части
	uint16_t len;
	char txt[MaxBodyLen];
} s_udhi_t;
#pragma pack(pop)

//------------------------------------------------------------------------------------------

extern char SMS_text[SMS_BUF_LEN];
extern char smsTMP[MAX_UART_BUF];

//------------------------------------------------------------------------------------------

extern int conv_ucs2_text(uint8_t *buffer_txt, char *fromik, uint8_t *udhi5, uint8_t prn);
extern void InitSMSList();
extern uint8_t PutSMSList(s_udhi_t *rec);
extern uint8_t LookAllPart(uint8_t total);
extern uint8_t ConcatSMS(char *buf, uint8_t total, uint16_t *sn, uint16_t *sl);
extern uint8_t getSMSTotalCounter();
extern int gsm7bit_to_text(int len_inbuff, uint8_t *inbuff, uint8_t *outbuff, int fl, uint8_t max_udl, uint8_t u_len);
extern int ucs2_to_utf8(char *buf_in, uint8_t *udl, uint8_t *utf8);
extern int8_t makeSMSString(const char *body, uint16_t *blen, char *fnum, uint16_t snum, char *buf, int max_len_buf);
extern void checkSMS(char *body, char *from);

//------------------------------------------------------------------------------------------


#endif

#endif /* __SMS_H__ */
