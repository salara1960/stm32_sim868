#include "sms.h"


#ifdef SET_SMS

static s_udhi_t sms_rec[maxSMSPart];
const char *sim_auth_num = "79097965036";
const char *eolin = "\r\n";
int TSINPART = 0;//from, date/time are present in part 1 sms only, if sms without udhi -> from, date/time not present
char SMS_text[SMS_BUF_LEN];
char smsTMP[MAX_UART_BUF];

const char *smsType[max_smsType] = {
	"+CMT: ",
	"+SCLASS0: ",
	"+CMGR: "
};

const char *tp[4] = {
	"GSM-7bit",
	"GSM-8bit",
	"UCS2",
	"???"
};

const char *type_name_a[9] = {
	"unknown",
	"International",
	"National",
	"Network",
	"Subscriber",
	"Alphanumeric",
	"Abbreviated",
	"reserved",
	"???"
};

//------------------------------------------------------------------------------------------
uint8_t hextobin(char st, char ml)
{
const char hex[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
uint8_t a = 255, b, c, i;

	for	(i = 0; i < 16; i++) { if (st == hex[i]) { b = i; break; } else b = 255; }
	for	(i = 0; i < 16; i++) { if (ml == hex[i]) { c = i; break; } else c = 255; }
	if ((b != 255) && (c != 255)) { b = b << 4;   a = b | c; }

	return a;
}
//------------------------------------------------------------------------------------------
void InitSMSList()
{
	memset((uint8_t *)&sms_rec[0], 0, sizeof(s_udhi_t) * maxSMSPart);
}
//-----------------------------------------------------------------------------
uint8_t PutSMSList(s_udhi_t *rec)
{
	if (!rec->part || rec->part > maxSMSPart) return 255;

	memcpy((uint8_t *)&sms_rec[rec->part - 1], (uint8_t *)rec, sizeof(s_udhi_t));

	return rec->part;
}
//-----------------------------------------------------------------------------
uint8_t LookAllPart(uint8_t total)
{
uint8_t ret = 0;

	if (total > maxSMSPart) return ret;

	for (uint8_t i = 0; i < total; i++) if (sms_rec[i].part) ret++;

	return ret;
}
//------------------------------------------------------------------------------
uint8_t ConcatSMS(char *buf, uint8_t total, uint16_t *sn, uint16_t *sl)
{

	if (total > maxSMSPart) return 0;

	uint16_t num = 0, len = 0, dl;
	uint8_t done = 0, i = 0;

	while (!done) {
		if (!num) {
			if (sms_rec[i].num) num = sms_rec[i].num;
		}
		if (!sms_rec[i].len) sms_rec[i].len = sprintf(sms_rec[i].txt, "--- part %u ---", i + 1);

		if ((len + sms_rec[i].len) > (SMS_BUF_LEN - 1)) {
			dl = SMS_BUF_LEN - 1 - len;
			done = 1;
		} else dl = sms_rec[i].len;

		memcpy(buf + len, sms_rec[i].txt, dl);
		len += dl;

		i++;
		if ((done) || (i >= total)) break;
	}
	*sn = num;
	*sl = len;

	return total;
}
//------------------------------------------------------------------------------
uint8_t getSMSTotalCounter()
{
uint8_t ret = 0;

	for (uint8_t i = 0; i < maxSMSPart; i++) {
		if (sms_rec[i].total) {
			ret = sms_rec[i].total;
			break;
		}
	}

	return ret;
}
//------------------------------------------------------------------------------
int gsm7bit_to_text(int len_inbuff, uint8_t *inbuff, uint8_t *outbuff, int fl, uint8_t max_udl, uint8_t u_len)
{
int dl_ind = 0, i = 0, shift = 1, lb = max_udl;
uint8_t a, b, prev = 0;
uint8_t *ps1 = inbuff;
uint8_t *ps2 = outbuff;
uint8_t words[4] = {0};

	if (!inbuff || !outbuff || !len_inbuff) return dl_ind;

    if (u_len > 0) lb -= (u_len + 1);
    lb--;

    memcpy(words, ps1 - 2, 2);
    b = hextobin(words[0], words[1]);
    if ((b > 0) && (fl)) {
    	*(uint8_t *)ps2 = b >> 1;
    	ps2++;
    	dl_ind++;
    }

    while (1) {
    	memcpy(words, ps1, 2);
    	a = hextobin(words[0], words[1]);
    	ps1 += 2;
    	i += 2;
    	b = a;
    	a <<= (uint8_t)(shift - 1);
    	a = (a | prev) & 0x7f;
    	prev = (b >> (8 - shift));
    	*(uint8_t *)ps2 = a;
    	ps2++;
    	dl_ind++;
    	if (shift != 7) shift++;
    	else {
    		*(uint8_t *)ps2 = prev;
    		ps2++;
    		dl_ind++;
    		shift = 1;
    		prev = 0;
    	}
    	if ((dl_ind > lb) || (i >= SMS_BUF_LEN - 1)) break;
    }

    return (dl_ind);
}
//----------------------------------------------------------------------------------
int conv_ucs2_text(uint8_t *buffer_txt, char *fromik, uint8_t *udhi5, uint8_t prn)
{
int ret = 0;
int tt, tt1 = 0, tt_n, tt1_n, len, i = 0, j = 0, k = 0, shift, it = 0, tzone, end_ind = 0;
char *ps1, *ps_begin, *ps3, *uk_start, *pss, *pss0, *ps_o, *ps_type, *ina2, *qik, *ps0;
char *uk_start7 = NULL;
char *ina = NULL;
char *ina0 = NULL;
char *ps_sta = NULL;
char *nachalo = NULL;
uint16_t dcs;
uint8_t a, a_n, b, b_n, c, prev, dl = 0, dl_ind, new_a;
uint8_t pdu_type = 0xff, type_num_a, len_num_a, user_data_len = 0;
uint8_t user_data_l = 0, udhi_len = 0, len_sca = 0, tp_mti = 0, tp_vpf = 0;
char words[5], chcs[12];
char words1[34], words2[34];
char stx[256];
int pdu, pack, ind_tp = 3, its_ok = 0, ofs = 0, with_udh = 0, flg = 0;
char udhi_str[32] = {0}, sca_str[32] = {0};
uint8_t udhi_4[5] = {0};
uint8_t buffer_temp[SMS_BUF_LEN] = {0};

	if ((ps0 = strstr((char *)buffer_txt, "\r\nOK\r\n")) != NULL) *(ps0) = '\0';

    len  =  strlen((char *)buffer_txt);

	if ((len < 7) || (len > 512)) {
		*buffer_txt = 0;
		return ret;
	} else {
		tt = pdu = 0;
		ps1 = (char *)&buffer_txt[0];
		ps0 = nachalo = ps1;
		ps_begin = ps_sta = ps1;
		memset(words, 0, sizeof(words));
		i = j = 0;
		ofs = 0;
		for (k = 0; k < max_smsType; k++) {
			ina0 = strstr(ps1, smsType[k]);////+CMT: ,,26 //+CMT: ,26 //+CMT: "",26 | "+CLASS0: " | "+CMGR: "
			if (ina0) {
				ina = ina0;
				ofs = strlen(smsType[k]);
				break;
			}
		}
		if (ina) {
			ina += ofs;
			ina2 = strstr(ina, "\r\n");
			if (ina2) {
				ina2 += 2;
				nachalo = ina2;//начало pdu !!!!!!!!!!!!!!!!!!!!!!!!!!
				if ((ina2 - ps1) < (len - ofs)) its_ok = 1;
				//---------------------------------------------------------
				k = 2;
				qik = strstr(ina, ",,");
				if (!qik) {
					qik = strstr(ina, "\",");
					if (!qik) {
						qik = strchr(ina, ',');
						if (qik) k = 1;
					} else k = 2;
				}
				if (qik) {
					qik += k;//указатель на начало длинны сообщения в байтах
					if ((ina2 - qik) > 1) i = ina2 - qik - 2;//!!!!!!!!!!!!!!! //1;//количество символов длинны самого pdu в байтах
					if ((i > 0) && (i < 4)) {
						memset(words, 0, sizeof(words));
						memcpy(words, qik, i);
						j = atoi(words);//собственно длинна pdu в байтах, указанная с сторке +CMGR: 1,,113
						j <<= 1;//собственно длинна pdu в символах в строке +CMGR: 1,,113
						k = strlen(ina2);//принято символов тела pdu
						if (k > j) {
							memset(words1, 0, sizeof(words1));
							memcpy(words1, ina2, 2);
							i = atoi(words1);//длинна номера sca
							i <<= 1;
							len_sca = i;
							ina2 += i + 2;
						}
					}
				}
				//---------------------------------------------------------
				ps1 = ps_begin = ina2;
				uk_start = ps1;
			}
		}

		if (!its_ok) {
			Report(false, "Sender number not found\r\n");
			uk_start7 = strstr(ps_begin, "00");
			if (uk_start7)  {
				if (uk_start7 == ps_sta) {
					its_ok = 1;
					nachalo = uk_start7 + 2;
					uk_start = nachalo;
				}
			}
		}


		if (its_ok) {
			memset(sca_str, 0, sizeof(sca_str));
			memset(words, 0, sizeof(words));
			if (nachalo) {
				memcpy(words, nachalo, 2);
				b = hextobin(words[0], words[1]);//sca_len
				if ((b > 0) && (b <= 7)) memcpy(sca_str, nachalo, (b + 1) << 1);
				if (!b) nachalo += 2;//1-й байт (длинна) = 0 -> номер смс-центра отсутствует
				   else nachalo += ((b + 1) << 1);//указатель на pdu_type
				memset(words, 0, sizeof(words));
				memcpy(words, nachalo, 2);
				pdu_type = hextobin(words[0], words[1]);//pdu_type
				if (pdu_type & 0x40) {
					with_udh = 1;
					memset(udhi_str, 0, sizeof(udhi_str));
					udhi_len = 0;
				}
				if (pdu_type & 1) tp_mti = 2; else tp_mti = 0;//есть MR или нет
				tp_vpf = ((pdu_type >> 3) & 3);//for submit sms
				uk_start = nachalo;
			}

			sprintf(stx, "CMGR/CMT_LEN=%s|%d, got_len=%d, PDU_TYPE=0x%02X", words, j, k, pdu_type);
			if (len_sca > 0) sprintf(stx+strlen(stx), ", SCA[%d]=%s,", len_sca, sca_str);
			if (with_udh) strcat(stx," With_UDHI");
					 else strcat(stx," Without_UDHI");
			//sprintf(stx+strlen(stx),"\r\nPDU:\r\n%s\r\n", uk_start);
			if (prn) Report(false, "%s\r\n", stx);

			ps1 = ps3 = uk_start;

			ps_type = uk_start + 4 + tp_mti;//указатель на тип номера
			memcpy(words, ps_type, 2);
			b = hextobin(words[0], words[1]);//тип номера
			b = (b & 0x70) >> 4; //нужны разряды bit6 bit5 bit4 - это тип номера A
			type_num_a = b;//тип номера А
			if (type_num_a > 8) type_num_a = 8;
			ps1 = ps1 + 2 + tp_mti;//указатель на кол-во символов в номере A - 0x0c
			memcpy(words, ps1, 2);
			a = hextobin(words[0], words[1]);
			if ((a & 1) == 1) a++;
			if (a > 32) a = 32;
			new_a = a;
			len_num_a = a;//длинна номера A !!!!!!!!!!!!!
			ps1 += 4;//указатель на начало номера A - 0x83
			memset(words1, 0, sizeof(words1));
			memcpy(words1, ps1, a);
			if (b != 5) {//надо переставить местами цифры номера
				j = 0;
				while (j < a) {
					c = words1[j];
					words1[j] = words1[j + 1];
					words1[j + 1] = c;
					j += 2;
				}
				if (words1[a - 1] == 'F') words1[a - 1] = ' ';
			} else {// номер закодирован в GSM-7bit - ПРИКИНЬ !  ВОТ УРОДЫ !!!
				j = 0; 	i = 0;
				memset(words2, 0, sizeof(words2));
				while (j < a) {
					words2[i] = hextobin(words1[j], words1[j + 1]);
					i++; j += 2;
				}
				//-------------- decoding num_a from GSM-7bit to KOI8-R -----------
				memset(words1, 0, sizeof(words1));
				tt_n = tt1_n = 0;
				dl_ind = prev = 0;
				dl = i;
				shift = 1;
				while (1) {
					a_n = words2[tt_n++];
					b_n = a_n;
					a_n <<= (uint8_t)(shift - 1);
					a_n = (a_n | prev) & 0x7f;
					words1[tt1_n++] = a_n;
					dl_ind++;
					prev = (b_n >> (8 - shift));
					if (shift != 7) {
						shift++;
					} else {
						words1[tt1_n++] = prev;
						dl_ind++;
						shift = 1;
						prev = 0;
					}
					if (dl_ind > dl) break;
				}
				//------------------------------------------------------------------------------------
				new_a = strlen(words1);
				a >>= 1;
			}

			end_ind = tt1 = 0;
			if (fromik) memset(fromik, 0, 32);    //макс-ая длинна номера А = 31
			if (b == 1) buffer_temp[tt1++] = '+'; //type_of_num_A = intern....
			ps_o = (char *)&buffer_temp[tt1];
			if (new_a > 0) {
				memcpy(ps_o, words1, new_a);
				tt1 += new_a;
				buffer_temp[tt1++] = 0x20;
			}
			memcpy(words, ps1 + len_num_a, 2);	  //p_id=hextobin(words[0], words[1]);
			pss = ps1 + len_num_a + 2;            //указатель на DCS

			memcpy(words, pss, 2);
			dcs = hextobin(words[0], words[1]);
			c = (dcs & 0x0c) >> 2;
			pdu = 0;//default
			switch (c) {
				case 0:
				case 1:
				case 3: pdu = 0; break;
				case 2: pdu = 1; break;
			}
			if (c > 2) c = 3;
			ind_tp = c;       //индекс типа кодировки, в которой получено сообщение bit3 bit2 в dcs
			if ((dcs & 0x20)) //dcs & 0x20 - признак компрессии, bit5 в dcs
				pack = 1;
			else
				pack = 0;

			k = strlen(words1);
			if (words1[k-1] == ' ') words1[k - 1] = '\0';
			if (fromik) {
				k = strlen(words1);
				if (k > lenFrom - 1) k = lenFrom - 1;
				memcpy(fromik, words1, k); //макс-ая длинна номера А = 31
			}

			sprintf(stx, "DCS=0x%02X, ENC[%d]=%s", dcs, ind_tp, tp[ind_tp]);
			if (new_a > 0) sprintf(stx+strlen(stx),", SENDER: type[%d]=%s number[%d]=%s",
													type_num_a, type_name_a[type_num_a], new_a, words1);
			if (pack) sprintf(stx+strlen(stx),", PACK=%d", pack);
			//
			//---------------------------------------------------
			//
			pss += 2; //указатель на начало области date/time
			uint8_t byte = 18;
			if (tp_vpf != 2) {
				memset(words1, 0, sizeof(words1));
				memcpy(words1, pss, 14); //14 символов для date/time
				j = 0;
				while (j < 14) {
					c = words1[j];
					words1[j] = words1[j + 1];
					words1[j + 1] = c;
					j += 2;
				}

				buffer_temp[tt1++] = 0x32; 	buffer_temp[tt1++] = 0x30; //20+год

				pss = &words1[0];	pss0 = (char *)&buffer_temp[tt1];	  memcpy(pss0, pss, 2);		tt1 += 2;   //год
				buffer_temp[tt1++] = '/';
				pss += 2; 		    pss0 = (char *)&buffer_temp[tt1];	  memcpy(pss0, pss, 2);		tt1 += 2;   //месяц
				buffer_temp[tt1++] = '/';
				pss += 2; 		    pss0 = (char *)&buffer_temp[tt1];     memcpy(pss0, pss, 2);		tt1 += 2;   //день
				buffer_temp[tt1++] = 0x20;

				pss += 2; 		    pss0 = (char *)&buffer_temp[tt1];     memcpy(pss0, pss, 2);		tt1 += 2;   //часы
				buffer_temp[tt1++] = ':';
				pss += 2; 		    pss0 = (char *)&buffer_temp[tt1];     memcpy(pss0, pss, 2);		tt1 += 2;   //минуты
				buffer_temp[tt1++] = ':';
				pss += 2; 		    pss0 = (char *)&buffer_temp[tt1];     memcpy(pss0, pss, 2);		tt1 += 2;   //секунды
				buffer_temp[tt1++] = '+';
				pss += 2;

				memset(chcs, 0, sizeof(chcs)); memcpy(chcs, pss, 2); tzone = atoi(chcs); tzone = (tzone * 15) / 60;
				memset(chcs, 0, sizeof(chcs)); sprintf(chcs, "%02d", tzone); pss = &chcs[0];
				pss0 = (char *)&buffer_temp[tt1]; memcpy(pss0, pss, 2); tt1 += 2;   //(+02)
			} else {
				pss += 2;
				byte = 6;
			}
			tt = (ps1 - ps0) + len_num_a + byte; //указатель на UDL - user data len
			memcpy(&buffer_temp[tt1], eolin, strlen(eolin)); tt1 += strlen(eolin);
			end_ind = tt1;
			//
			ps3 = (char *)&buffer_txt[tt];
			memset(words, 0, sizeof(words));
			memcpy(words, ps3, 2);
			dl = hextobin(words[0], words[1]); //длинна тела сообщения
			user_data_l = user_data_len = dl;
			sprintf(stx+strlen(stx),", UDL=%d[%s]\r\n", user_data_len, words);
			if (prn) Report(false, stx);

			tt += 2; //индекс на начало текста сообщения или на начало udhi
		}//if (its_ok)
		//
		//----------------------------------------------------------------
		//
		dl_ind = 0;
		if (pdu == 1) { //тело сообщения в UCS2

			if (with_udh) { //Это часть сообщения
				ps1 = (char *)&buffer_txt[tt];
				memset(words, 0, sizeof(words));
				memcpy(words, ps1, 2);
				udhi_len = hextobin(words[0], words[1]);
				if (udhi_len <= 32) {
					memcpy(udhi_str, ps1 + 2, udhi_len << 1);
					tt += ((udhi_len + 1) << 1); //NEW BODY POINTER (index)
					dl -= udhi_len + 1;
					user_data_len = dl;
				}
			}

			dl_ind = ucs2_to_utf8((char *)&buffer_txt[tt], &user_data_len, &buffer_temp[tt1]);
			tt1 += dl_ind;
		} else {//if (pdu == 1)
			switch (ind_tp) {
				case 0 ://7 bit encoding
					k = 0;
					if (with_udh) {
						ps1 = (char *)&buffer_txt[tt];
						memset(words, 0, sizeof(words));
						memcpy(words, ps1, 2);
						udhi_len = hextobin(words[0], words[1]);
						if (udhi_len <= 32) {
							memcpy(udhi_str, ps1 + 2, udhi_len << 1);
							tt += ((udhi_len + 1) << 1);//NEW BODY POINTER
							k = 7 - (udhi_len + 1);
							tt += k << 1;
							if (k > 0) flg = k; else flg = 0;
							dl -= udhi_len + 1;
							k = dl * 7;
							dl = k >> 3;
							if (k % 8) dl++;
							user_data_len = dl;
						}
					} else {
						flg = 0;
					}

					dl_ind = gsm7bit_to_text(dl, &buffer_txt[tt], &buffer_temp[end_ind], flg, user_data_l, udhi_len);

					tt1 += dl_ind;
				break;
				case 1://8 bit encoding
					if (with_udh) {
						ps1 = (char *)&buffer_txt[tt];
						memcpy(words, ps1, 2);
						udhi_len = hextobin(words[0], words[1]);
						if (udhi_len <= 32) {
							memcpy(udhi_str, ps1 + 2, udhi_len << 1);
							tt += ((udhi_len + 1) << 1);//NEW BODY POINTER
							dl -= udhi_len + 1;
							user_data_len = dl;
						}
					}

					while (1) {
						ps1 = (char *)&buffer_txt[tt];
						memcpy(words, ps1, 2);
						buffer_temp[tt1++] = hextobin(words[0], words[1]);
						tt += 2;
						dl_ind++;
						if ((tt >= len) || (dl_ind >= dl)) break;
					}
				break;
			}//switch (ind_tp)
		}//else -> 7 ! 8 bit encoding

		buffer_temp[tt1] = 0;

		if (with_udh) {
			if (udhi_len >= 5) {
				udhi_4[0] = 1; //tp
				a = hextobin(udhi_str[2], udhi_str[3]);
				b = (udhi_len << 1);
				if (a == 4) { //2 байта на номер смс
					b -= 8;
					udhi_4[1] = hextobin(udhi_str[b], udhi_str[b + 1]); //num_1
					b += 2;
				} else { //1 байт на номер смс
					udhi_4[1] = 0; //num_1
					b -= 6;
				}
				udhi_4[2] = hextobin(udhi_str[b],     udhi_str[b + 1]); //num_2
				udhi_4[3] = hextobin(udhi_str[b + 2], udhi_str[b + 3]); //total
				udhi_4[4] = hextobin(udhi_str[b + 4], udhi_str[b + 5]); //part
			}

			a = udhi_4[1];   udhi_4[1] = udhi_4[2];   udhi_4[2] = a;//swapbytes in sms_num
			memcpy(udhi5, udhi_4, 5);

			if (TSINPART) {
				end_ind = 0;
			} else {
				if (udhi_4[4] < 2) end_ind = 0;
			}
		} else {
			if (TSINPART) end_ind = 0;
		}

		if (end_ind > SMS_BUF_LEN - 1) end_ind = 0;

		it = SMS_BUF_LEN - 1 - end_ind;

		memset(buffer_txt, 0, SMS_BUF_LEN);
		memcpy(buffer_txt, &buffer_temp[end_ind], it);

		if (with_udh && prn) Report(false,"UDH(%d): [%s]\r\n", udhi_len, udhi_str);

		ret = tt1 - end_ind;//strlen((char *)buffer_txt);//dl_ind;

	}

 	return ret;
}
//-----------------------------------------------------------------------------------------
int ucs2_to_utf8(char *buf_in, uint8_t *udl, uint8_t *utf8)
{
int ret = 0, i = 0, len;
uint8_t a, b;
char *ptr = buf_in;
uint8_t *out = utf8;
uint16_t ucs2;


	if (!udl) len = strlen(buf_in) >> 2; else len = *udl >> 1;

    while (i < len) {
    	a = hextobin(*ptr, *(ptr + 1));   ptr += 2;
    	b = hextobin(*ptr, *(ptr + 1));   ptr += 2;
    	ucs2 = a;   ucs2 <<= 8;   ucs2 |= b;
    	if (ucs2 < 0x80) {
    		*out++ = (uint8_t)ucs2;
    		ret++;
    	} else {
    		*out++ = (uint8_t)((ucs2 >> 6)   | 0xC0);
    		*out++ = (uint8_t)((ucs2 & 0x3F) | 0x80);
    		ret += 2;
    	}
    	i++;
    }

    return ret;
}
//-----------------------------------------------------------------------------------------
int8_t makeSMSString(const char *body, uint16_t *blen, char *fnum, uint16_t snum, char *buf, int max_len_buf)
{
int8_t ret = -1;

#ifdef SET_JFES
    uint8_t i = 0;
    jfes_value_t *obj = jfes_create_object_value(jconf);
    if (obj) {
    	jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, ++infCounter), Items[i++], 0);//"SeqNum"
    	jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, "SMS", 0), Items[i++], 0);//"MsgType"
    	if (strlen(devID)) jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, devID, 0), Items[i++], 0);//"DevID"
    	jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, dev_name, 0), Items[i++], 0);//"DevName"
    	jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, sim_num, 0), Items[i++], 0);//"SimNumber"
    	jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, get_secCounter()), Items[i++], 0);//"DevTime"
    	jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, xPortGetFreeHeapSize()), Items[i++], 0);//"FreeMem"
    	if (setDate) {
    	#ifdef SET_RTC_TMR
    	            uint32_t ep = getSecRTC(&hrtc);
    	#else
    	            uint32_t ep = get_extDate();
    	#endif
    	            jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, ep), Items[i++], 0);//"EpochTime"
    	}
    	jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, snum), "smsNumber", 0);
    	jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, fnum, 0), "fromNumber", 0);
    	jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, body, 0), "smsBody", 0);

        jfes_size_t stx_size  = (jfes_size_t)max_len_buf;
        jfes_value_to_string(obj, buf, &stx_size, 1);
        *(buf + stx_size) = '\0';
        strcat(buf, "\r\n");

        jfes_free_value(jconf, obj);

        ret = 0;
    }

#else
    uint16_t body_len = *blen;
    strcpy(buf, "{\r\n");
    int len = 4;

    sprintf(buf+strlen(buf), "\t\"SeqNum\": %lu,\r\n", ++infCounter);
    sprintf(buf+strlen(buf), "\t\"MsgType\": \"SMS\",\r\n");
    sprintf(buf+strlen(buf), "\t\"DevID\": \"%s\",\r\n", devID);
    sprintf(buf+strlen(buf), "\t\"DevName\": \"%s\",\r\n", dev_name);
    sprintf(buf+strlen(buf), "\t\"SimNumber\": \"%s\",\r\n", sim_num);
    sprintf(buf+strlen(buf), "\t\"DevTime\": %lu,\r\n", get_secCounter());
    sprintf(buf+strlen(buf), "\t\"FreeMem\": %u,\r\n",xPortGetFreeHeapSize());
    if (setDate) {
#ifdef SET_RTC_TMR
    	uint32_t ep = getSecRTC(&hrtc);
#else
    	uint32_t ep = get_extDate();
#endif
    	sprintf(buf+strlen(buf), "\t\"EpochTime\": %lu,\r\n", ep);
    }
    sprintf(buf+strlen(buf), "\t\"smsNumber\": %u,\r\n", snum);
    sprintf(buf+strlen(buf), "\t\"fromNumber\": \"%s\",\r\n", fnum);
    len += strlen(buf);
    if ((len + body_len + 16) > max_len_buf) body_len = max_len_buf - len - 16;
    sprintf(buf+strlen(buf), "\t\"smsBody\": \"%.*s\"\r\n}\r\n", body_len, body);

    ret = 0;

#endif

    *blen = (uint16_t)strlen(buf);

    return ret;
}
//------------------------------------------------------------------------------------------
void checkSMS(char *body, char *from)//  check : command present in sms ?
{

	if (!strstr(from, sim_auth_num)) {
		Report(true, "Error : sms from '%s' number (auth number '%s')\r\n", from, sim_auth_num);
		return;
	}
	//
	if (strstr(body, ":INF")) {
		flags.inf = 1;
	} else if (strstr(body, ":GET")) {
		flags.msg_begin = 1;
	} else if (strstr(body, ":CON")) {
		flags.connect = 1;
		con_dis = true;
	} else if (strstr(body, ":DIS")) {
		flags.disconnect = 1;
		con_dis = false;
	} else if (strstr(body, ":VIO")) {
		flags.vio = 1;
	} else if (strstr(body, ":ON")) {
		evt_gsm = 1;
	} else if (strstr(body, ":OFF")) {
		evt_gsm = 2;
	} else if (strstr(body, ":RESTART")) {
		if (LoopAll) {
			flags.restart = 1;
		} else {
#ifdef SET_OLED_I2C
			i2c_ssd1306_clear();
#endif
#ifdef SET_OLED_SPI
			spi_ssd1306_clear();
#endif
			NVIC_SystemReset();
		}
	}
}

//------------------------------------------------------------------------------------------

#endif
