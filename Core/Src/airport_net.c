#include "airport_net.h"
#include "clamp_control.h"
#include "socket.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* WY-32C-B8R schematic: PB10 INT, PB11 RESET, PB12 CS, PB13..15 SPI2. */
static SPI_HandleTypeDef net_spi;
static uint8_t ready, connected, owned, spi_error;
static uint32_t last_rx;
static char input[96], output[1024];
static uint16_t input_length, output_length, output_sent;
static uint8_t discard;

static void select_chip(void) { HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET); }
static void deselect_chip(void) { HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET); }
static uint8_t spi_read(void)
{
    uint8_t dummy=0xff, value=0xff;
    if(HAL_SPI_TransmitReceive(&net_spi,&dummy,&value,1,2)!=HAL_OK) spi_error=1;
    return value;
}
static void spi_write(uint8_t value)
{
    if(HAL_SPI_Transmit(&net_spi,&value,1,2)!=HAL_OK) spi_error=1;
}
static void append(const char *text)
{
    size_t n=strlen(text);
    if(n>sizeof(output)-output_length) {
        if(owned) Clamp_StopAll("NET_TX_FULL");
        owned=0;
        return;
    }
    memcpy(output+output_length,text,n); output_length+=(uint16_t)n;
}
static void reply(int ok,const char *name)
{
    char line[64];
    snprintf(line,sizeof(line),"%s %s\r\n",ok?"OK":"ERR",name); append(line);
}
static unsigned long tenths(uint32_t counts)
{
    return (unsigned long)((uint64_t)counts*10U/LR400_COUNTS_PER_MM);
}
static void status(void)
{
    for(uint8_t i=1;i<=2;i++) {
        const ClampAxis *a=Clamp_Get(i);
        char line[240];
        unsigned long d=tenths(a->distance), s=tenths(a->start), e=tenths(a->end), t=tenths(a->target);
        snprintf(line,sizeof(line),
            "AXIS %u distance_mm=%lu.%lu valid=%u state=%u start_mm=%lu.%lu end_mm=%lu.%lu target_mm=%lu.%lu fault=%s\r\n",
            i,d/10,d%10,a->valid && HAL_GetTick()-a->sample_tick<300U,a->state,
            s/10,s%10,e/10,e%10,t/10,t%10,a->fault);
        append(line);
    }
    append("OK STATUS\r\n");
}
static uint8_t healthy(void)
{
    return Clamp_Get(1)->state!=3 && Clamp_Get(2)->state!=3;
}
static void handle(char *line)
{
    if(!strcmp(line,"STATUS")) {status(); return;}
    if(!strcmp(line,"PING")) {reply(1,"PING"); return;}
    ClampCommand cmd={0};
    if(!strcmp(line,"STOP")) {
        cmd.type=CLAMP_STOP; Clamp_Execute(&cmd); owned=0; reply(1,"STOP"); return;
    }
    if(!strcmp(line,"CLEAR")) {
        cmd.type=CLAMP_CLEAR; reply(Clamp_Execute(&cmd),"CLEAR"); return;
    }
    if(!strcmp(line,"GRIP") || !strcmp(line,"RELEASE")) {
        cmd.type=!strcmp(line,"GRIP")?CLAMP_GRIP:CLAMP_RELEASE;
        int same=owned && healthy();
        for(uint8_t i=1;i<=2;i++) {
            const ClampAxis *a=Clamp_Get(i);
            if(a->target!=(cmd.type==CLAMP_GRIP?a->end:a->start)) same=0;
        }
        if(same) {reply(1,line); return;}
        if(!healthy()) {reply(0,line); return;}
        /* Direction reversal is an explicit stop followed by a fresh target. */
        Clamp_StopAll(NULL); owned=0;
        int ok=Clamp_Execute(&cmd);
        if(ok) owned=1;
        reply(ok,line); return;
    }
    unsigned axis; char start_text[32],end_text[32],extra;
    if(sscanf(line,"ENDPOINTS %u %31s %31s %c",&axis,start_text,end_text,&extra)==3) {
        char *start_tail,*end_tail;
        float start=strtof(start_text,&start_tail),end=strtof(end_text,&end_tail);
        int ok=axis>=1 && axis<=2 && !*start_tail && !*end_tail && isfinite(start) && isfinite(end) &&
            Clamp_Get(1)->state==0 && Clamp_Get(2)->state==0 &&
            Clamp_SetEndpoints((uint8_t)axis,start,end);
        if(ok) owned=0;
        reply(ok,"ENDPOINTS"); return;
    }
    reply(0,"COMMAND");
}
static void reset_connection(const char *fault)
{
    if(owned && fault) Clamp_StopAll(fault);
    owned=connected=discard=0;
    input_length=output_length=output_sent=0;
    close(0);
}
void Airport_Net_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_SPI2_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
    GPIO_InitTypeDef g={0};
    g.Pin=GPIO_PIN_11|GPIO_PIN_12; g.Mode=GPIO_MODE_OUTPUT_PP;
    g.Speed=GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(GPIOB,&g);
    g.Pin=GPIO_PIN_10; g.Mode=GPIO_MODE_INPUT; g.Pull=GPIO_PULLUP; HAL_GPIO_Init(GPIOB,&g);
    g.Pin=GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15; g.Mode=GPIO_MODE_AF_PP;
    g.Pull=GPIO_NOPULL; g.Alternate=GPIO_AF5_SPI2; HAL_GPIO_Init(GPIOB,&g);
    net_spi.Instance=SPI2;
    net_spi.Init.Mode=SPI_MODE_MASTER;
    net_spi.Init.Direction=SPI_DIRECTION_2LINES;
    net_spi.Init.DataSize=SPI_DATASIZE_8BIT;
    net_spi.Init.CLKPolarity=SPI_POLARITY_LOW;
    net_spi.Init.CLKPhase=SPI_PHASE_1EDGE;
    net_spi.Init.NSS=SPI_NSS_SOFT;
    net_spi.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16;
    net_spi.Init.FirstBit=SPI_FIRSTBIT_MSB;
    net_spi.Init.TIMode=SPI_TIMODE_DISABLE;
    net_spi.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;
    net_spi.Init.CRCPolynomial=7;
    net_spi.Init.CRCLength=SPI_CRC_LENGTH_DATASIZE;
    net_spi.Init.NSSPMode=SPI_NSS_PULSE_DISABLE;
    if(HAL_SPI_Init(&net_spi)!=HAL_OK) {printf("NET SPI_INIT_FAILED\r\n"); return;}
    HAL_Delay(2); HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET); HAL_Delay(100);
    /* Only the main loop touches SPI. Leave IRQs enabled for motor/RS485 timing. */
    reg_wizchip_cs_cbfunc(select_chip,deselect_chip);
    reg_wizchip_spi_cbfunc(spi_read,spi_write);
    if(getVERSIONR()!=4 || spi_error) {printf("NET W5500_NOT_FOUND\r\n"); return;}
    uint8_t memory[8]={2,2,2,2,2,2,2,2};
    if(wizchip_init(memory,memory)!=0) {printf("NET INIT_FAILED\r\n"); return;}
    uint32_t uid=HAL_GetUIDw0()^HAL_GetUIDw1()^HAL_GetUIDw2();
    wiz_NetInfo info={.mac={2,0x47,(uint8_t)(uid>>24),(uint8_t)(uid>>16),(uint8_t)(uid>>8),(uint8_t)uid},
        .ip={192,168,8,103},.sn={255,255,255,0},.gw={0,0,0,0},.dns={0,0,0,0},.dhcp=NETINFO_STATIC};
    wizchip_setnetinfo(&info);
    setRTR(2000); setRCR(2);
    ready=1;
    printf("NET W5500 192.168.8.103/24 TCP 5000 ready\r\n");
}
void Airport_Net_Task(void)
{
    if(!ready) return;
    if(spi_error) {
        if(owned) Clamp_StopAll("NET_SPI");
        owned=0; ready=0; return;
    }
    uint8_t state=getSn_SR(0);
    if(state==SOCK_CLOSED) {
        if(connected) reset_connection("NET_LOST");
        socket(0,Sn_MR_TCP,AIRPORT_NET_PORT,SF_IO_NONBLOCK); return;
    }
    if(state==SOCK_INIT) {listen(0); return;}
    if(state!=SOCK_ESTABLISHED) {
        if(connected || state==SOCK_CLOSE_WAIT) reset_connection("NET_LOST");
        return;
    }
    if(!connected) {
        connected=1; last_rx=HAL_GetTick(); input_length=output_length=output_sent=0;
        /* Only the configured industrial PC may issue actuator commands. */
        uint8_t peer[4]; getSn_DIPR(0,peer);
        if(peer[0]!=192 || peer[1]!=168 || peer[2]!=8 || peer[3]!=102) {
            reset_connection(NULL); return;
        }
    }
    if(HAL_GetTick()-last_rx>AIRPORT_NET_TIMEOUT_MS) {reset_connection("NET_TIMEOUT"); return;}
    if(output_sent<output_length) {
        int32_t n=send(0,(uint8_t *)output+output_sent,output_length-output_sent);
        if(n<0) {reset_connection("NET_LOST"); return;}
        output_sent+=(uint16_t)n;
        if(output_sent==output_length) output_sent=output_length=0;
    }
    /* Drain queued replies before reading more requests, keeping RAM bounded. */
    if(output_length) return;
    uint8_t bytes[96];
    int32_t n=recv(0,bytes,sizeof(bytes));
    if(n<0) {reset_connection("NET_LOST"); return;}
    if(n>0) last_rx=HAL_GetTick();
    for(int32_t j=0;j<n;j++) {
        uint8_t ch=bytes[j];
        if(ch=='\r' || ch=='\n') {
            if(!discard && input_length) {input[input_length]=0; handle(input);}
            input_length=0; discard=0;
        } else if(ch<32 || ch>126 || input_length>=sizeof(input)-1) {
            if(!discard) {
                if(owned) Clamp_StopAll("NET_COMMAND");
                owned=0; reply(0,"COMMAND_TOO_LONG_OR_INVALID");
            }
            discard=1; input_length=0;
        } else if(!discard) input[input_length++]=(char)ch;
    }
}
