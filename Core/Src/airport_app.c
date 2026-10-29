#include "airport_app.h"
#include "clamp_control.h"
#include "airport_net.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SENSOR_FUNCTION 4U
#define SENSOR_TIMEOUT 120U
#define JOG_PULSES_PER_REV 800U /* Must match the driver's DIP switches. */
typedef struct {
    UART_HandleTypeDef *uart;
    uint8_t byte;
    volatile uint8_t data[32], count, overflow, error;
    volatile uint32_t last;
    uint32_t sent;
    volatile uint8_t waiting;
    volatile uint32_t rx_bytes, uart_errors;
    uint32_t requests, tx_errors, accepted, empty, rejected;
    uint8_t last_frame[32], last_count;
} Sensor;
static Sensor sensors[2];
static uint8_t console_byte;
static volatile uint8_t ring[256];
static volatile uint16_t head, tail;
static volatile uint8_t console_error;
static uint8_t stream=1;

int __io_putchar(int ch)
{
    uint8_t b=(uint8_t)ch;
    (void)HAL_UART_Transmit(&huart1,&b,1,10);
    return ch;
}
static uint16_t crc16(const uint8_t *p,uint32_t n)
{
    uint16_t c=0xffff;
    while(n--) {
        c^=*p++;
        for(uint8_t j=0;j<8;j++) c=c&1?(c>>1)^0xa001:c>>1;
    }
    return c;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if(uart==&huart1) {
        uint16_t next=(head+1)&255;
        if(next!=tail) {ring[head]=console_byte; head=next;}
        else console_error=1;
        (void)HAL_UART_Receive_IT(uart,&console_byte,1);
    } else for(uint8_t i=0;i<2;i++) if(uart==sensors[i].uart) {
        Sensor *s=&sensors[i];
        s->rx_bytes++;
        if(s->waiting) {
            if(s->count<sizeof(s->data)) s->data[s->count++]=s->byte;
            else s->overflow=1;
            s->last=HAL_GetTick();
        }
        (void)HAL_UART_Receive_IT(uart,&s->byte,1);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if(uart==&huart1) console_error=1;
    else for(uint8_t i=0;i<2;i++) if(uart==sensors[i].uart) {
        sensors[i].error=1; sensors[i].uart_errors++;
    }
}
static void sensor_task(void)
{
    static uint8_t selected;
    static uint32_t next;
    Sensor *s=&sensors[selected];
    uint32_t now=HAL_GetTick();
    for(uint8_t i=0;i<2;i++) if(sensors[i].error) {
        sensors[i].error=0;
        (void)HAL_UART_AbortReceive(sensors[i].uart);
        __HAL_UART_CLEAR_OREFLAG(sensors[i].uart);
        (void)HAL_UART_Receive_IT(sensors[i].uart,&sensors[i].byte,1);
        Clamp_Sensor(i+1,0,0);
    }
    if(s->waiting) {
        if((s->count && now-s->last>=5) || now-s->sent>=SENSOR_TIMEOUT) {
            uint8_t f[32],n,bad;
            uint32_t key=__get_PRIMASK(); __disable_irq();
            n=s->count; bad=s->overflow; s->waiting=0;
            for(uint8_t j=0;j<n;j++) f[j]=s->data[j];
            __set_PRIMASK(key);
            s->last_count=n; memcpy(s->last_frame,f,n);
            uint8_t valid=!bad && n==9 && f[0]==1 && f[1]==SENSOR_FUNCTION && f[2]==4;
            if(valid) valid=crc16(f,7)==((uint16_t)f[7]|((uint16_t)f[8]<<8));
            uint32_t raw=valid?((uint32_t)f[3]<<24)|((uint32_t)f[4]<<16)|((uint32_t)f[5]<<8)|f[6]:0;
            if(valid) s->accepted++; else if(!n) s->empty++; else s->rejected++;
            Clamp_Sensor(selected+1,raw,valid);
            selected^=1; next=now+10;
        }
    } else if((int32_t)(now-next)>=0) {
        uint8_t q[8]={1,SENSOR_FUNCTION,0,0,0,2,0,0};
        uint16_t c=crc16(q,6); q[6]=(uint8_t)c; q[7]=(uint8_t)(c>>8);
        s->count=0; s->overflow=0; s->waiting=1; s->sent=now;
        s->requests++;
        if(HAL_UART_Transmit(s->uart,q,8,20)!=HAL_OK) {
            s->tx_errors++;
            s->waiting=0; Clamp_Sensor(selected+1,0,0); selected^=1; next=now+10;
        }
    }
}
/* Fixed-point text avoids enabling printf's large floating-point formatter. */
static void print_mm(uint32_t counts)
{
    uint32_t tenths=(uint32_t)((uint64_t)counts*10U/LR400_COUNTS_PER_MM);
    printf("%lu.%lu",(unsigned long)(tenths/10U),(unsigned long)(tenths%10U));
}
static void status(void)
{
    for(uint8_t i=1;i<=2;i++) {
        const ClampAxis *a=Clamp_Get(i);
        printf("AXIS %u distance_mm=",i); print_mm(a->distance);
        printf(" valid=%u age=%lu state=%u start_mm=",a->valid && HAL_GetTick()-a->sample_tick<300U,
            (unsigned long)(HAL_GetTick()-a->sample_tick),a->state);
        print_mm(a->start); printf(" end_mm="); print_mm(a->end);
        printf(" target_mm="); print_mm(a->target);
        printf(" tol_mm="); print_mm(a->tolerance);
        printf(" cal=%u fault=%s close_dir=%u close_sign=%d endpoints=%u\r\n",
            a->calibrated,a->fault,a->close_level,a->close_sign,a->endpoints);
        if(a->fault_axis) {
            printf("FAULT source_axis=%u trigger_mm=",a->fault_axis); print_mm(a->fault_distance);
            printf(" target_mm="); print_mm(a->fault_target); printf("\r\n");
        }
    }
}
static int number(const char *s,int32_t *value)
{
    if(!s || !*s) return 0;
    char *end; long n=strtol(s,&end,10);
    if(*end || n < -1000000L || n > 1000000L) return 0;
    *value=(int32_t)n; return 1;
}
/* Decimal motor-shaft degrees, up to 3 decimal places. Round to the nearest
 * physical pulse; use 64-bit arithmetic to reject overflow before narrowing. */
static int angle_pulses(const char *s,uint32_t *value)
{
    const uint64_t max_mdeg=(uint64_t)UINT32_MAX*360000U/JOG_PULSES_PER_REV;
    uint64_t whole=0, fraction=0, scale=100;
    if(!s || *s<'0' || *s>'9') return 0;
    while(*s>='0' && *s<='9') {
        whole=whole*10U+(unsigned)(*s++-'0');
        if(whole>max_mdeg/1000U) return 0;
    }
    if(*s=='.') {
        s++;
        if(*s<'0' || *s>'9') return 0;
        while(*s>='0' && *s<='9') {
            if(!scale) return 0;
            fraction+=(unsigned)(*s++-'0')*scale; scale/=10U;
        }
    }
    uint64_t mdeg=whole*1000U+fraction;
    if(*s || mdeg>max_mdeg) return 0;
    uint64_t pulses=(mdeg*JOG_PULSES_PER_REV+180000U)/360000U;
    if(!pulses || pulses>UINT32_MAX) return 0;
    *value=(uint32_t)pulses; return 1;
}
static void command(char *line)
{
    char *words[7]; uint8_t n=0;
    char *word=strtok(line," \t");
    while(word && n<7) {words[n++]=word; word=strtok(NULL," \t");}
    if(!n) return;
    if(!strcmp(words[0],"DIAG") && n==1) {
        for(uint8_t i=0;i<2;i++) {
            Sensor *s=&sensors[i];
            printf("RS485 axis=%u uart=%s baud=%lu req=%lu rx=%lu ok=%lu empty=%lu bad=%lu txerr=%lu uarterr=%lu last=",
                i+1,s->uart==&huart2?"USART2":"UART4",(unsigned long)s->uart->Init.BaudRate,
                (unsigned long)s->requests,(unsigned long)s->rx_bytes,(unsigned long)s->accepted,
                (unsigned long)s->empty,(unsigned long)s->rejected,(unsigned long)s->tx_errors,(unsigned long)s->uart_errors);
            for(uint8_t j=0;j<s->last_count;j++) printf("%02X ",s->last_frame[j]);
            printf("\r\n");
        }
        return;
    }
    if(!strcmp(words[0],"HELP") && n==1) {
        printf("STATUS | STREAM 0/1 | STOP | CLEAR [axis] | UNTEACH axis\r\n"
               "JOG axis dir(0/1) degrees hz(100..20000); 800 pulses/rev; degrees up to 3 decimals\r\n"
               "CAL axis close_dir(0/1) close_sign(-1/1)\r\n"
               "START axis mm | END axis mm | TEACH_START axis | TEACH_END axis\r\n"
               "TOL axis mm | GRIP [axis] | RELEASE [axis]\r\n"); return;
    }
    if(!strcmp(words[0],"STATUS") && n==1) {status(); return;}
    int32_t v[5]={0};
    uint32_t jog_pulses=0;
    uint8_t jog=!strcmp(words[0],"JOG") && n==5;
    uint8_t distance_cmd=n==3 && (!strcmp(words[0],"START") || !strcmp(words[0],"END") || !strcmp(words[0],"TOL"));
    if(n>6) {printf("ERR syntax\r\n"); return;}
    for(uint8_t i=1;i<n;i++) {
        if(jog && i==3) {
            if(!angle_pulses(words[i],&jog_pulses)) {printf("ERR angle\r\n"); return;}
        } else if(distance_cmd && i==2) {
            char *end; float mm=strtof(words[i],&end); uint32_t counts;
            if(*end || !isfinite(mm) || !Clamp_MmToCounts(mm,&counts)) {printf("ERR distance_mm\r\n"); return;}
            v[i-1]=(int32_t)counts;
        } else if(!number(words[i],&v[i-1])) {printf("ERR number\r\n"); return;}
    }
    if(!strcmp(words[0],"STREAM") && n==2 && (v[0]==0 || v[0]==1)) {
        stream=(uint8_t)v[0]; printf("OK STREAM\r\n"); return;
    }
    ClampCommand c={0}; uint8_t matched=1;
    if(!strcmp(words[0],"STOP") && n==1) c.type=CLAMP_STOP;
    else if(!strcmp(words[0],"CLEAR") && n<=2) c.type=CLAMP_CLEAR;
    else if(!strcmp(words[0],"GRIP") && n<=2) c.type=CLAMP_GRIP;
    else if(!strcmp(words[0],"RELEASE") && n<=2) c.type=CLAMP_RELEASE;
    else if(jog) {c.type=CLAMP_JOG;c.sign=v[1];c.value=jog_pulses;c.speed=(uint32_t)v[3];}
    else if(!strcmp(words[0],"CAL") && n==4) {c.type=CLAMP_CALIBRATE;c.value=(uint32_t)v[1];c.sign=v[2];}
    else if(!strcmp(words[0],"START") && n==3) {c.type=CLAMP_SET_START;c.value=(uint32_t)v[1];}
    else if(!strcmp(words[0],"END") && n==3) {c.type=CLAMP_SET_END;c.value=(uint32_t)v[1];}
    else if(!strcmp(words[0],"TOL") && n==3) {c.type=CLAMP_TOLERANCE;c.value=(uint32_t)v[1];}
    else if(!strcmp(words[0],"TEACH_START") && n==2) c.type=CLAMP_TEACH_START;
    else if(!strcmp(words[0],"TEACH_END") && n==2) c.type=CLAMP_TEACH_END;
    else if(!strcmp(words[0],"UNTEACH") && n==2) c.type=CLAMP_UNTEACH;
    else matched=0;
    if(v[0]<0 || v[0]>2) matched=0;
    c.axis=(uint8_t)v[0];
    printf("%s %s\r\n",matched && Clamp_Execute(&c)?"OK":"ERR",words[0]);
}
static void console_task(void)
{
    static char line[96]; static uint8_t length,discard;
    if(console_error) {
        console_error=0; Clamp_StopAll("CONSOLE_RX");
        (void)HAL_UART_AbortReceive(&huart1); __HAL_UART_CLEAR_OREFLAG(&huart1);
        (void)HAL_UART_Receive_IT(&huart1,&console_byte,1);
        tail=head; length=0; discard=1;
    }
    for(uint16_t budget=0; budget<96 && tail!=head; budget++) {
        uint8_t ch=ring[tail]; tail=(tail+1)&255;
        if(ch=='\r' || ch=='\n') {
            if(!discard && length) {line[length]=0; command(line);}
            length=0; discard=0;
        } else if(ch>=32 && ch<=126 && !discard) {
            if(length<sizeof(line)-1) line[length++]=(char)ch;
            else {discard=1; length=0; Clamp_StopAll("COMMAND_TOO_LONG");}
        }
    }
}
void Airport_App_Init(void)
{
    setvbuf(stdout,NULL,_IONBF,0);
    Clamp_Init();
    Airport_Net_Init();
    /* Verified by individual motor tests: motor 1 changes USART2 distance;
     * motor 2 changes UART4 distance. Telemetry numbering now follows motors. */
    sensors[0].uart=&huart2; sensors[1].uart=&huart4;
    for(uint8_t i=0;i<2;i++) (void)HAL_UART_Receive_IT(sensors[i].uart,&sensors[i].byte,1);
    (void)HAL_UART_Receive_IT(&huart1,&console_byte,1);
    printf("AIRPORT clamp v9 STOPPED; continuous=1000Hz; median5, arrival_latch; distance=mm; close_dir=0; 115200-8N1; HELP\r\n");
}
void Airport_App_Task(void)
{
    static uint32_t printed;
    Clamp_Task(); sensor_task(); Clamp_Task(); console_task(); Clamp_Task();
    Airport_Net_Task(); Clamp_Task();
    if(stream && HAL_GetTick()-printed>=200U) {
        printed=HAL_GetTick();
        for(uint8_t i=1;i<=2;i++) {
            const ClampAxis *a=Clamp_Get(i);
            printf("LR400-%u distance_mm=",i); print_mm(a->distance);
            printf(" valid=%u state=%u fault=%s\r\n",a->valid && HAL_GetTick()-a->sample_tick<300,
                a->state,a->fault);
        }
    }
}
