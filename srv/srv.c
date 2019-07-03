//************************************************************************
//            tcp server for testing client (stm32_sim868)
//************************************************************************
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <stdbool.h>
#include <time.h>
#include <pthread.h>
#include <string.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <sys/un.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <ctype.h>
#include <sys/msg.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <dirent.h>

#include <jansson.h>

#define LEN64 64
#define LEN128 128
#define LEN256 256
#define LEN1K 1024
#define buf_size (LEN1K << 1)
#define TIME_STR_LEN LEN64
#define tmr_wait_def 60
#define tcp_port_def 9192

//--------------------------------------------------------------------


unsigned int tmr_wait = tmr_wait_def;
pid_t pid_main;
FILE *pid_fp = NULL;
const char *pid_name = "srv.pid";

const unsigned int sizeofint = sizeof(unsigned int);

unsigned char QuitAll=0;
unsigned char SIGHUPs = 1;
unsigned char SIGTERMs = 1;
unsigned char SIGINTs = 1;
unsigned char SIGKILLs = 1;
unsigned char SIGSEGVs = 1;
unsigned char SIGABRTs = 1;
unsigned char SIGSYSs = 1;
unsigned char SIGTRAPs = 1;

static char dt_str[TIME_STR_LEN] = {0};

const char *SeqNum = "SeqNum";
//const char *SensSN = "SensSeqNum";
//const char *GpsSN  = "GpsSeqNum";
const char *DataSN = "DataSeqNum";
const char *InfSN = "InfSeqNum";

//------------------------------------------------------------------------
char *TimeNowPrn(char *ts)
{
struct timeval tvl;
int i_hour, i_min, i_sec, i_day, i_mes, mil;

    gettimeofday(&tvl, NULL);
    time_t ct = tvl.tv_sec;
    struct tm *ctimka = localtime(&ct);
    i_hour = ctimka->tm_hour; i_min = ctimka->tm_min;     i_sec = ctimka->tm_sec;
    i_day  = ctimka->tm_mday; i_mes = ctimka->tm_mon + 1; mil   = (int)(tvl.tv_usec / 1000);;
    sprintf(ts,"%02d.%02d %02d:%02d:%02d.%03d | ", i_day, i_mes, i_hour, i_min, i_sec, mil);

    return ts;
}
//-----------------------------------------------------------------------
void print_cdr(const char *st, unsigned char dt)
{
    if (dt) printf("%s", TimeNowPrn(dt_str));

    printf("%s", st);
}
//-----------------------------------------------------------------------
static unsigned int get_timer_sec(unsigned int t)
{
    return ((unsigned int)time(NULL) + t);
}
//-----------------------------------------------------------------------
static int check_delay_sec(unsigned int t)
{
    if ((unsigned int)time(NULL) >= t)  return 1; else return 0;
}
//--------------------  function for recive SIGNAL from system -----------
void GetSignal_(int sig)
{
int out = 0;
char st[LEN64];

    switch (sig) {
        case SIGHUP:
            print_cdr("\tSignal SIGHUP\n",1);
        break;
        case SIGKILL:
            if (SIGKILLs) {
                SIGKILLs = 0;
                print_cdr("\tSignal SIGKILL\n",1);
                out = 1;
            }
        break;
        case SIGPIPE:
            print_cdr("\tSignal SIGPIPE\n",1);
        break;
        case SIGTERM:
            if (SIGTERMs) {
                SIGTERMs = 0;
                print_cdr("\tSignal SIGTERM\n",1);
                out = 1;
            }
        break;
        case SIGINT:
            if (SIGINTs) {
                SIGINTs = 0;
                print_cdr("\tSignal SIGINT\n",1);
                out = 1;
            }
        break;
        case SIGSEGV:
            if (SIGSEGVs) {
                SIGSEGVs = 0;
                print_cdr("\tSignal SIGSEGV\n",1);
                out = 1;
            }
        break;
        case SIGABRT:
            if (SIGABRTs) {
                SIGABRTs = 0;
                print_cdr("\tSignal SIGABRT\n",1);
                out = 1;
            }
        break;
        case SIGSYS:
            if (SIGSYSs) {
                SIGSYSs = 0;
                print_cdr("\tSignal SIGSYS\n",1);
                out = 1;
            }
        break;
        case SIGTRAP:
            if (SIGTRAPs) {
                SIGTRAPs = 0;
                print_cdr("\tSignal SIGTRAP\n",1);
                out = 1;
            }
        break;
            default : {
                sprintf(st,"\tUNKNOWN signal %d",sig);
                print_cdr(st, 1);
            }
    }

    if (out) QuitAll = out;
}
//-----------------------------------------------------------------------
int CheckPack(const char *js, char *packName)
{
int ret = -1;
json_t *obj = NULL, *tmp = NULL;
json_error_t err;
char *stx = NULL;

    if (strstr(js, SeqNum) == NULL) return ret;

    obj = json_loads(js, 0, &err);
    if (!obj) {
        stx = (char *)calloc(1, strlen(err.text) + 64);
        if (stx) {
            sprintf(stx,"CheckPack parser error : on line %d: %s\n", err.line, err.text);
            print_cdr(stx, 1);
            free(stx);
        }
        return ret;
    }

    tmp = json_object_get(obj, DataSN);
    if (tmp) {
        ret = json_integer_value(tmp);
        strcpy(packName, DataSN);
    } else {
        tmp = json_object_get(obj, InfSN);
        if (tmp) {
            ret = json_integer_value(tmp);
            strcpy(packName, InfSN);
        } else strcpy(packName, "Unknown");
    }

    if (obj) json_decref(obj);

    return ret;
}
//----------------------------------------------------------------------------------
void *cli_nitka(void *arg)
{
char *uki = NULL;
fd_set read_Fds;
struct timeval cli_tv;
int lenrecv = 0, tmp = 0, ready = 0, Vixod = 0, res = 0, seqn;
unsigned long int tmr, pack_number = 0;
char from_client[buf_size];
char chap[LEN256];
char numName[64];

    int client = *(int *)arg;

    sprintf(chap,"Start thread with socket %d. Wait data %u sec. ...\n", client, tmr_wait);
    print_cdr(chap, 1);

    uki = from_client;
    tmr = get_timer_sec(tmr_wait);

    while (!Vixod) {
        cli_tv.tv_sec  = 0;
        cli_tv.tv_usec = 50000;
        FD_ZERO(&read_Fds);
        FD_SET(client, &read_Fds);
        if (select(client + 1, &read_Fds, NULL, NULL, &cli_tv) > 0) {

            tmp = recv(client, uki, 1, MSG_DONTWAIT);
            if (!tmp) {//client hangup
                Vixod = 1;
                if (lenrecv > 0) ready = 1;
            } else if (tmp > 0) {
                lenrecv += tmp;
                uki     += tmp;
                if (strstr(from_client, "exit")) {
                    ready = 1;
                    res = 1;
                    Vixod = 1;
                } else
                if ( (strstr(from_client, "}\r\n")) || (lenrecv >= buf_size - 2) )  ready = 1;
            }

            if (ready) {
                ready = 0;
                //if (from_client[strlen(from_client)-1] != '\n') strcat(from_client, "\n");
                uki = strstr(from_client, "}\r\n");
                if (uki) *(uki + 1) = 0;
                strcat(from_client, "\n");
                print_cdr(from_client, 1);
                //
                numName[0] = 0;
                seqn = CheckPack(from_client, numName);
                //
                sprintf(from_client, "{\"PackNumber\":%lu,\"%s\":%d}\n", ++pack_number, numName, seqn);
                send(client, from_client, strlen(from_client), MSG_DONTWAIT);
                print_cdr(from_client, 1);

                memset(from_client, 0, buf_size);
                lenrecv = 0;
                uki = from_client;
                tmr = get_timer_sec(tmr_wait);
            }//ready

        }//select

        if (QuitAll) break;
        else {
            if (check_delay_sec(tmr)) {
                Vixod = 1;
                res = 2;
            }
        }

    }//while

    if (client) {
        shutdown(client, SHUT_RDWR);
        close(client);
    }

    sprintf(chap, "Close thread with socket %d ", client);
    switch (res) {
        case 1 : sprintf(chap+strlen(chap), "[Client closed connection]\n"); break;
        case 2 : sprintf(chap+strlen(chap), "[Timeout %d sec]\n", tmr_wait); break;
            default : sprintf(chap+strlen(chap), "[Global interrupt]\n");
    }
    print_cdr(chap, 1);

    pthread_exit(NULL);

}
//**********************************************************************
//**********************************************************************
//**********************************************************************
int main (int argc, char *argv[])
{
time_t ttm;
fd_set Fds;
char *stri = NULL, *abra = NULL;
unsigned short tcp_port = tcp_port_def;
struct sockaddr_in srv_conn, cli_conn;
socklen_t srvlen, clilen;
struct timeval mytv;
pthread_t tid;
pthread_attr_t threadAttr;
struct sigaction Act, OldAct;
char stril[LEN64] = {0};
char chap[LEN256] = {0};
//char server_port[LEN64] = {0};
int connsocket = -1, client = -1, resa, Vixod = 0, on = 1;

/*
    if (argc < 2) {
        printf("start server : ./srv 9292 45\n\n");
        return 1;
    }
*/

    pid_main = getpid();
    pid_fp = fopen(pid_name, "w");
    if (!pid_fp) {
        printf("Unable to create pid file %s: %s\n", pid_name, strerror(errno));
        return 1;
    } else {
        fprintf(pid_fp, "%i\n", (int)pid_main);
        fclose(pid_fp);
    }

    if (argc > 1) {
        resa = atoi(argv[1]);
        if ((resa > 0) && (resa < 0xffff)) tcp_port = resa;
    }
    if (argc > 2) {
        resa = atoi(argv[2]);
        if ((resa > 0) && (resa <= 120)) tmr_wait = resa;
    }

    //------------  set signals route function ------------------
    memset((unsigned char *)&Act, 0, sizeof(struct sigaction));
    memset((unsigned char *)&OldAct, 0, sizeof(struct sigaction));
    Act.sa_handler = &GetSignal_;
    Act.sa_flags   = 0;
    sigaction(SIGPIPE, &Act, &OldAct);
    sigaction(SIGHUP,  &Act, &OldAct);
    sigaction(SIGSEGV, &Act, &OldAct);
    sigaction(SIGTERM, &Act, &OldAct);
    sigaction(SIGABRT, &Act, &OldAct);
    sigaction(SIGINT,  &Act, &OldAct);
    sigaction(SIGSYS,  &Act, &OldAct);
    sigaction(SIGKILL, &Act, &OldAct);
    sigaction(SIGTRAP, &Act, &OldAct);

    ttm = time(NULL);
    abra = ctime(&ttm);
    abra[strlen(abra) - 1] = 0;
    sprintf(chap,"%s Start server\n", abra);
    print_cdr(chap, 1);

    //--------------------------------------------------------------------

    while (!QuitAll) {

        Vixod = 0;
        connsocket = socket(AF_INET, SOCK_STREAM, 6);

        if (connsocket < 0) {
            sprintf(chap,"ERROR: open socket (%d)\n",connsocket);
            print_cdr(chap, 1);
            return 1;
        }

        on = 1;
        if (setsockopt(connsocket, SOL_SOCKET, SO_REUSEADDR,(const char *) &on, sizeof(on))) {
            sprintf(chap,"ERROR: Setsockopt - SO_REUSEADDR (%d)\n",connsocket);
            print_cdr(chap, 1);
            return 1;
        }

        srvlen = sizeof(struct sockaddr_in);
        memset(&srv_conn, 0, srvlen);
        srv_conn.sin_family      = AF_INET;
        srv_conn.sin_addr.s_addr = htonl(INADDR_ANY);
        srv_conn.sin_port        = htons(tcp_port);

        if (bind(connsocket, (struct sockaddr *) &srv_conn, srvlen) == -1) {
            sprintf(chap,"ERROR: Bind [port %d].\n", tcp_port);
            print_cdr(chap, 1);
            goto out_of_job;
        }

        if (listen(connsocket, 3) == -1) {
            sprintf(chap,"ERROR: Listen [port %d].\n", tcp_port);
            print_cdr(chap, 1);
            goto out_of_job;
        }

        fcntl(connsocket, F_SETFL, (fcntl(connsocket, F_GETFL)) | O_NONBLOCK);

        sprintf(chap,"Listen client [port %d timeout %u sec.]...\n", tcp_port, tmr_wait);
        print_cdr(chap, 1);


        pthread_attr_init(&threadAttr);
        pthread_attr_setdetachstate(&threadAttr, PTHREAD_CREATE_DETACHED);

        clilen = sizeof(struct sockaddr_in);

        while (!Vixod) {
            if (QuitAll) break;
            resa = 0;
            FD_ZERO(&Fds);
            FD_SET(connsocket,&Fds);
            mytv.tv_sec = 0;
            mytv.tv_usec = 50000;
            resa = select(connsocket + 1, &Fds, NULL, NULL, &mytv);
            if (resa > 0) {
                client = accept(connsocket, (struct sockaddr *) &cli_conn, &clilen);
                if (client > 0) {
                    fcntl(client, F_SETFL, (fcntl(client, F_GETFL)) | O_NONBLOCK);//set client socket to nonblocking mode
                    memset(stril, 0, LEN64);
                    stri = (char *)inet_ntoa(cli_conn.sin_addr);
                    sprintf(stril, "%s", stri);
                    sprintf(chap, "New client %s:%u online (socket %d)\n", stril, htons(cli_conn.sin_port), client);
                    print_cdr(chap, 1);
                   //------------------------------------------------------------
                   if (pthread_create(&tid, &threadAttr, cli_nitka, &client)) {//start client's thread to service the request
                        pthread_attr_destroy(&threadAttr);;
                        sprintf(chap,"Error start thread for socket %d\n", client);
                        print_cdr(chap, 1);
                        if (client) {
                            close(client);
                            client = -1;
                        }
                    }
                    //------------------------------------------------------------
                }//if (client > 0)
            }//if (resa > 0)
        }//while (!Vixod)

        pthread_attr_destroy(&threadAttr);

out_of_job:


        if (client > 0) {
            shutdown(client, SHUT_RDWR);
            close(client);
        }
        if (connsocket > 0) {
            shutdown(connsocket, SHUT_RDWR);
            close(connsocket);
        }

    }//while(1<2)


    if (pid_main) unlink(pid_name);

    return 0;
}

