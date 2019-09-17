//************************************************************************
//            tcp server for testing client (stm32_sim868)
//************************************************************************
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
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
#include <arpa/inet.h>
#include <ctype.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include <jansson.h>


#define WITH_KERNEL_TIMER


#define LEN256 256
#define LEN1K 1024
#define LEN2K 2048
#define buf_size LEN2K
#define TIME_STR_LEN 64
#define tcp_port_def 9192


#define tmr_wait_def 90

/*
#define get_timer(tm) ((unsigned int)time(NULL) + tm)
#define check_delay(tm) ((unsigned int)time(NULL) >= tm ? true : false)
*/

//--------------------------------------------------------------------


uint32_t tmr_wait = tmr_wait_def;
pid_t pid_main;
FILE *pid_fp = NULL;
const char *pid_name = "srv.pid";
const char *the_log = "logs.txt";
int fd_log = -1;
#ifdef WITH_KERNEL_TIMER
    int fd_timer = -1;
    const char *tmr_name = "/dev/tmr";
#endif

const uint32_t sizeofint = sizeof(unsigned int);

uint8_t QuitAll=0;
uint8_t SIGHUPs = 1;
uint8_t SIGTERMs = 1;
uint8_t SIGINTs = 1;
uint8_t SIGKILLs = 1;
uint8_t SIGSEGVs = 1;
uint8_t SIGABRTs = 1;
uint8_t SIGSYSs = 1;
uint8_t SIGTRAPs = 1;

static char dt_str[TIME_STR_LEN] = {0};

const char *SeqNum = "SeqNum";

//------------------------------------------------------------------------
char *TimeNowPrn(char *ts)
{
struct timeval tvl;

    gettimeofday(&tvl, NULL);
    struct tm *ctimka = localtime(&tvl.tv_sec);
    sprintf(ts, "%02d.%02d %02d:%02d:%02d.%03d | ",
                ctimka->tm_mday, ctimka->tm_mon + 1,
                ctimka->tm_hour, ctimka->tm_min, ctimka->tm_sec, (int)(tvl.tv_usec / 1000));

    return ts;
}
//-----------------------------------------------------------------------
void print_msg(uint32_t dt, const char *fmt, ...)
{
size_t len = LEN2K;
char *st = (char *)calloc(1, len);

    if (st) {
        int dl = 0, sz;
        va_list args;
        if (dt) dl = sprintf(st, "%s", TimeNowPrn(dt_str));
        sz = dl;
        va_start(args, fmt);
        sz += vsnprintf(st + dl, len - dl, fmt, args);
        va_end(args);
        printf("%s", st);
        if (fd_log) write(fd_log, st, strlen(st));
        free(st);
    }

}
//-----------------------------------------------------------------------
uint32_t get_timer(uint32_t t)
{
uint32_t ret = 0;
#ifdef WITH_KERNEL_TIMER
    if (fd_timer > 0) {
        uint8_t one[8];
        ret = (uint32_t)(read(fd_timer, one, 0) + t);
    }
#else
    ret = (uint32_t)(time(NULL) + t);
#endif
    return ret;
}
//-----------------------------------------------------------------------
int check_delay(uint32_t t)
{
uint32_t ret = 1;
#ifdef WITH_KERNEL_TIMER
    if (fd_timer > 0) {
        uint8_t one[8];
        if (read(fd_timer, one, 0) < t) ret = 0;
    }
#else
    if ((uint32_t)time(NULL) < t) ret = 0;
#endif
    return ret;
}
//--------------------  function for recive SIGNAL from system -----------
void GetSignal_(int sig)
{
int out = 0;

    switch (sig) {
        case SIGHUP:
            print_msg(1, "\tSignal SIGHUP\n");
        break;
        case SIGKILL:
            if (SIGKILLs) {
                SIGKILLs = 0;
                print_msg(1, "\tSignal SIGKILL\n");
                out = 1;
            }
        break;
        case SIGPIPE:
            print_msg(1, "\tSignal SIGPIPE\n");
        break;
        case SIGTERM:
            if (SIGTERMs) {
                SIGTERMs = 0;
                print_msg(1, "\tSignal SIGTERM\n");
                out = 1;
            }
        break;
        case SIGINT:
            if (SIGINTs) {
                SIGINTs = 0;
                print_msg(1, "\tSignal SIGINT\n");
                out = 1;
            }
        break;
        case SIGSEGV:
            if (SIGSEGVs) {
                SIGSEGVs = 0;
                print_msg(1, "\tSignal SIGSEGV\n");
                out = 1;
            }
        break;
        case SIGABRT:
            if (SIGABRTs) {
                SIGABRTs = 0;
                print_msg(1, "\tSignal SIGABRT\n");
                out = 1;
            }
        break;
        case SIGSYS:
            if (SIGSYSs) {
                SIGSYSs = 0;
                print_msg(1, "\tSignal SIGSYS\n");
                out = 1;
            }
        break;
        case SIGTRAP:
            if (SIGTRAPs) {
                SIGTRAPs = 0;
                print_msg(1, "\tSignal SIGTRAP\n");
                out = 1;
            }
        break;
            default : {
                print_msg(1, "\tUNKNOWN signal %d", sig);
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

    if (strstr(js, SeqNum) == NULL) return ret;

    obj = json_loads(js, 0, &err);
    if (!obj) {
        print_msg(1, "CheckPack parser error : on line %d: %s\n", err.line, err.text);
        return ret;
    }

    tmp = json_object_get(obj, SeqNum);
    if (tmp) {
        ret = json_integer_value(tmp);
        strcpy(packName, SeqNum);
    } else strcpy(packName, "Unknown");

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
uint32_t tmr, pack_number = 0;
char from_client[buf_size];
char chap[LEN256];
char numName[64];

    int client = *(int *)arg;

    print_msg(1, "Start thread with socket %d. Wait data %u sec. ...\n", client, tmr_wait);

    uki = from_client;

    tmr = get_timer(tmr_wait);

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
                print_msg(1, from_client);
                //
                numName[0] = 0;
                seqn = CheckPack(from_client, numName);
                //
                sprintf(from_client, "{\"PackNumber\":%u,\"%s\":%d}\n", ++pack_number, numName, seqn);
                send(client, from_client, strlen(from_client), MSG_DONTWAIT);
                print_msg(1, from_client);

                memset(from_client, 0, buf_size);
                lenrecv = 0;
                uki = from_client;
                tmr = get_timer(tmr_wait);
            }//ready

        }//select

        if (QuitAll) break;
        else {
            if (check_delay(tmr)) {
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
    print_msg(1, chap);

    pthread_exit(NULL);

}
//**********************************************************************
//**********************************************************************
//**********************************************************************
int main (int argc, char *argv[])
{
time_t ttm;
fd_set Fds;
char *abra = NULL;
uint16_t tcp_port = tcp_port_def;
struct sockaddr_in srv_conn, cli_conn;
socklen_t srvlen, clilen;
struct timeval mytv;
pthread_t tid;
pthread_attr_t threadAttr;
struct sigaction Act, OldAct;
int connsocket = -1, client = -1, resa, Vixod = 0, on = 1;
uint8_t errs = 0;
uint32_t tik = 0;


    fd_log = open(the_log, O_WRONLY | O_APPEND | O_CREAT, 0664);
    if (fd_log < 0) {
        print_msg(1, "%s Can't open %s file (%d)\n", TimeNowPrn(dt_str), the_log, fd_log);
        return 1;
    }

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
    if (argc > 2) {//max 180 sec
        resa = atoi(argv[2]);
        if ((resa > 0) && (resa <= 180)) tmr_wait = resa;
    }

    //------------  set signals route function ------------------
    memset((uint8_t *)&Act,    0, sizeof(struct sigaction));
    memset((uint8_t *)&OldAct, 0, sizeof(struct sigaction));
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
    print_msg(1, "%s Start server\n", abra);

#ifdef WITH_KERNEL_TIMER
    //--------------------  open TMR device  -----------------------------
    if ((fd_timer = open(tmr_name, O_RDWR) ) < 0) {
        print_msg(1, "Can't open device '%s' (%s) - > Main done.\n", tmr_name, strerror(errno));
        if (pid_main) unlink(pid_name);
        if (fd_log > 0) close(fd_log);
        return 1;
    } else {
        tik = get_timer(0);
        print_msg(1, "Open device '%s' OK (sec=%u)\n", tmr_name, tik);
    }
    //--------------------------------------------------------------------
#endif

    while (!QuitAll) {

        Vixod = 0;
        connsocket = socket(AF_INET, SOCK_STREAM, 6);

        if (connsocket < 0) {
            print_msg(1, "ERROR: open socket (%d)\n",connsocket);
            return 1;
        }

        on = 1;
        if (setsockopt(connsocket, SOL_SOCKET, SO_REUSEADDR,(const char *) &on, sizeof(on))) {
            print_msg(1, "ERROR: Setsockopt - SO_REUSEADDR (%d)\n",connsocket);
            return 1;
        }

        srvlen = sizeof(struct sockaddr_in);
        memset(&srv_conn, 0, srvlen);
        srv_conn.sin_family      = AF_INET;
        srv_conn.sin_addr.s_addr = htonl(INADDR_ANY);
        srv_conn.sin_port        = htons(tcp_port);

        if (bind(connsocket, (struct sockaddr *) &srv_conn, srvlen) == -1) {
            print_msg(1, "ERROR: Bind [port %d].\n", tcp_port);
            errs = 1;
            goto out_of_job;
        }

        if (listen(connsocket, 3) == -1) {
            print_msg(1, "ERROR: Listen [port %d].\n", tcp_port);
            errs = 1;
            goto out_of_job;
        }

        fcntl(connsocket, F_SETFL, (fcntl(connsocket, F_GETFL)) | O_NONBLOCK);

        print_msg(1, "Listen client [port %d timeout %u sec.]...\n", tcp_port, tmr_wait);

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
                    tik = get_timer(0);
                    print_msg(1, "New client %s:%u online (socket %d) (sec=%u)\n",
                                 (char *)inet_ntoa(cli_conn.sin_addr), htons(cli_conn.sin_port), client, tik);
                   //------------------------------------------------------------
                   if (pthread_create(&tid, &threadAttr, cli_nitka, &client)) {//start client's thread to service the request
                        pthread_attr_destroy(&threadAttr);;
                        print_msg(1, "Error start thread for socket %d\n", client);
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

        if (errs) break;


    }//while(1<2)

    print_msg(1, "Main done (sec=%u)\n", get_timer(0));

#ifdef WITH_KERNEL_TIMER
    if (fd_timer > 0) close(fd_timer);
#endif

    if (pid_main) unlink(pid_name);

    if (fd_log > 0) close(fd_log);

    return 0;
}

