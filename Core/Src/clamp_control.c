#include "clamp_control.h"
#include <string.h>
#include <math.h>

/* P14: pin4 PL1 PA15=PUL1, pin3 PL2 PF9=DIR1,
 *      pin2 3PWM/OUT PE0=PUL2, pin1 4PWM/OUT PE1=DIR2.
 * Page 11 schematic is authoritative; spreadsheet swaps PL3/PL4 MCU pins.
 * MOSFET sinks current: GPIO high asserts the driver's optocoupler. */
#define TICK_HZ 40000U /* Two edges per 20kHz pulse. */
#define FRESH_MS 300U
#define MOVE_TIMEOUT_MS CLAMP_MOVE_TIMEOUT_MS
typedef struct {
    GPIO_TypeDef *pulse_port, *dir_port;
    uint16_t pulse_pin, dir_pin;
    volatile uint32_t left, phase, period, lease;
    volatile uint8_t active, high, expired, continuous;
} PulseAxis;
static PulseAxis pulse[2] = {
    {.pulse_port=GPIOA, .dir_port=GPIOF, .pulse_pin=GPIO_PIN_15, .dir_pin=GPIO_PIN_9},
    {.pulse_port=GPIOE, .dir_port=GPIOE, .pulse_pin=GPIO_PIN_0, .dir_pin=GPIO_PIN_1}
};
static ClampAxis axes[2];
static void endpoint_calibration(ClampAxis *a);
int Clamp_MmToCounts(float mm, uint32_t *counts)
{
    double value=(double)mm*LR400_COUNTS_PER_MM;
    if(!counts || !LR400_COUNTS_PER_MM || !isfinite(value) || value<0.5 || value>1000000.0) return 0;
    *counts=(uint32_t)(value+0.5); return 1;
}
float Clamp_CountsToMm(uint32_t counts)
{
    return LR400_COUNTS_PER_MM ? (float)counts/(float)LR400_COUNTS_PER_MM : NAN;
}
static uint32_t lock(void) { uint32_t p=__get_PRIMASK(); __disable_irq(); return p; }
static void unlock(uint32_t p) { __set_PRIMASK(p); }
static void pin(GPIO_TypeDef *p, uint16_t n, uint8_t high) { p->BSRR=high ? n : (uint32_t)n<<16; }
static uint8_t fresh(const ClampAxis *a) { return a->valid && HAL_GetTick()-a->sample_tick < FRESH_MS; }
static void stop_pulse(uint8_t i)
{
    uint32_t k=lock(); pulse[i].active=0; pulse[i].left=0;
    pulse[i].high=0; pulse[i].expired=0; pulse[i].continuous=0;
    pin(pulse[i].pulse_port,pulse[i].pulse_pin,0); unlock(k);
}
void Clamp_StopAll(const char *reason)
{
    for(uint8_t i=0;i<2;i++) {
        stop_pulse(i);
        axes[i].state=reason ? 3 : 0;
        axes[i].fault=reason ? reason : "NONE";
        if(!reason) { axes[i].fault_axis=0; axes[i].fault_distance=0; axes[i].fault_target=0; }
    }
}
static void motion_fault(uint8_t i,const char *reason)
{
    uint32_t distance=axes[i].distance,target=axes[i].target;
    Clamp_StopAll(reason);
    for(uint8_t j=0;j<2;j++) {
        axes[j].fault_axis=i+1;
        axes[j].fault_distance=distance; axes[j].fault_target=target;
    }
}
/* count == 0 selects continuous positioning; JOG always supplies a finite count. */
static void start_pulse(uint8_t i,uint8_t dir,uint32_t count,uint32_t hz)
{
    PulseAxis *p=&pulse[i];
    uint32_t k=lock();
    pin(p->dir_port,p->dir_pin,dir);
    p->period=TICK_HZ/hz; p->phase=TICK_HZ/500U; /* 2ms DIR setup */
    p->left=count; p->continuous=count==0;
    p->high=0; p->expired=0; p->active=1; p->lease=TICK_HZ/10;
    unlock(k);
}
void Clamp_PulseIRQ(void)
{
    for(uint8_t i=0;i<2;i++) {
        PulseAxis *p=&pulse[i];
        if(!p->active) continue;
        /* Main-loop lease: pulses also stop if foreground processing stalls. */
        if(!p->lease || --p->lease==0) {
            pin(p->pulse_port,p->pulse_pin,0); p->active=0; p->left=0; p->high=0; p->expired=1;
            continue;
        }
        if(p->phase && --p->phase) continue;
        if(p->high) {
            pin(p->pulse_port,p->pulse_pin,0); p->high=0;
            if(!p->continuous && !p->left) {p->active=0; continue;}
            p->phase=p->period-p->period/2;
        } else {
            if(!p->continuous && !p->left) {p->active=0; continue;}
            pin(p->pulse_port,p->pulse_pin,1); p->high=1;
            if(!p->continuous) p->left--;
            p->phase=p->period/2;
        }
    }
}
void Clamp_Init(void)
{
    static const float starts[2] = {CLAMP_AXIS1_START_MM, CLAMP_AXIS2_START_MM};
    static const float ends[2] = {CLAMP_AXIS1_END_MM, CLAMP_AXIS2_END_MM};
    memset(axes,0,sizeof(axes));
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOE_CLK_ENABLE(); __HAL_RCC_GPIOF_CLK_ENABLE();
    GPIO_InitTypeDef g={0}; g.Mode=GPIO_MODE_OUTPUT_PP; g.Pull=GPIO_PULLDOWN;
    g.Speed=GPIO_SPEED_FREQ_HIGH;
    for(uint8_t i=0;i<2;i++) {
        pin(pulse[i].pulse_port,pulse[i].pulse_pin,0); pin(pulse[i].dir_port,pulse[i].dir_pin,0);
        g.Pin=pulse[i].pulse_pin; HAL_GPIO_Init(pulse[i].pulse_port,&g);
        g.Pin=pulse[i].dir_pin; HAL_GPIO_Init(pulse[i].dir_port,&g);
        uint8_t has_start=Clamp_MmToCounts(starts[i],&axes[i].start);
        uint8_t has_end=Clamp_MmToCounts(ends[i],&axes[i].end);
        axes[i].tolerance=1;
        (void)Clamp_MmToCounts(CLAMP_DEFAULT_TOLERANCE_MM,&axes[i].tolerance);
        axes[i].fault="NONE";
        axes[i].close_level=CLAMP_CLOSE_DIR;
        axes[i].endpoints=(has_start?1U:0U)|(has_end?2U:0U);
        endpoint_calibration(&axes[i]);
    }
    /* TIM6 is owned by this module; it is NOT the HAL timebase. APB1 timer
     * frequency includes the x2 multiplier when the APB prescaler is not /1. */
    __HAL_RCC_TIM6_CLK_ENABLE();
    uint32_t hz=HAL_RCC_GetPCLK1Freq();
    if((RCC->CFGR & RCC_CFGR_PPRE1)!=0) hz*=2;
    TIM6->CR1=0; TIM6->PSC=hz/1000000U-1; TIM6->ARR=1000000U/TICK_HZ-1;
    TIM6->EGR=TIM_EGR_UG; TIM6->SR=0; TIM6->DIER=TIM_DIER_UIE;
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn,3,0); HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1=TIM_CR1_CEN;
    /* Halt pulse timebase during debugger stops. */
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM6_STOP;
}
const ClampAxis *Clamp_Get(uint8_t axis) { return axis>=1 && axis<=2 ? &axes[axis-1] : NULL; }
void Clamp_Sensor(uint8_t axis,uint32_t raw,uint8_t valid)
{
    if(axis<1 || axis>2) return;
    ClampAxis *a=&axes[axis-1];
    /* Zero and out-of-range/sentinel encodings are not accepted as positions. */
    a->valid=valid && raw>0 && raw<=1000000U;
    if(a->valid) {
        /* Five-sample median rejects isolated vibration/reflection spikes. */
        if(HAL_GetTick()-a->sample_tick>=FRESH_MS) a->sample_count=a->sample_index=0;
        a->samples[a->sample_index]=raw; a->sample_index=(a->sample_index+1U)%5U;
        if(a->sample_count<5) a->sample_count++;
        uint32_t sorted[5];
        for(uint8_t j=0;j<a->sample_count;j++) sorted[j]=a->samples[j];
        for(uint8_t j=1;j<a->sample_count;j++) {
            uint32_t v=sorted[j]; uint8_t k=j;
            while(k && sorted[k-1]>v) {sorted[k]=sorted[k-1]; k--;}
            sorted[k]=v;
        }
        a->distance=sorted[a->sample_count/2U]; a->sample_tick=HAL_GetTick(); a->sequence++;
    }
    if(!a->valid && (axes[0].state==2 || axes[1].state==2))
        Clamp_StopAll("SENSOR");
}
static uint8_t ready(ClampAxis *a)
{
    return LR400_COUNTS_PER_MM && a->state==0 && fresh(a) && a->calibrated && a->endpoints==3 &&
        a->end!=a->start && ((a->end>a->start)?1:-1)==a->close_sign;
}
/* Endpoint labels provide the feedback polarity without guessing sensor mounting.
 * START is open; END is closed. Re-teach both after moving a sensor. */
static void endpoint_calibration(ClampAxis *a)
{
    if(a->endpoints==3) {
        a->calibrated=a->start!=a->end;
        a->close_sign=a->end>a->start?1:-1;
    }
}
static int valid_raw(uint32_t raw) { return raw>=1U && raw<=1000000U; }
static int set_endpoint(ClampAxis *a, uint8_t end, uint32_t raw)
{
    if(!valid_raw(raw) || a->state!=0) return 0;
    if(end) {a->end=raw; a->endpoints|=2U;}
    else {a->start=raw; a->endpoints|=1U;}
    endpoint_calibration(a);
    return 1;
}
int Clamp_SetStart(uint8_t axis, float mm)
{
    uint32_t raw;
    return Clamp_MmToCounts(mm,&raw) && axis>=1 && axis<=2 ? set_endpoint(&axes[axis-1],0,raw) : 0;
}
int Clamp_SetEnd(uint8_t axis, float mm)
{
    uint32_t raw;
    return Clamp_MmToCounts(mm,&raw) && axis>=1 && axis<=2 ? set_endpoint(&axes[axis-1],1,raw) : 0;
}
int Clamp_SetEndpoints(uint8_t axis, float start_mm, float end_mm)
{
    uint32_t start,end;
    if(!Clamp_MmToCounts(start_mm,&start) || !Clamp_MmToCounts(end_mm,&end)) return 0;
    if(axis<1 || axis>2 || !valid_raw(start) || !valid_raw(end) || start==end || axes[axis-1].state!=0) return 0;
    ClampAxis *a=&axes[axis-1];
    a->start=start; a->end=end; a->endpoints=3U; endpoint_calibration(a);
    return a->calibrated;
}
int Clamp_Execute(const ClampCommand *c)
{
    if(!c || c->axis>2) return 0;
    if(c->type==CLAMP_STOP) {Clamp_StopAll(NULL); return 1;}
    uint8_t lo=c->axis?c->axis-1:0, hi=c->axis?c->axis:2;
    if(c->type==CLAMP_CLEAR) {
        for(uint8_t i=lo;i<hi;i++) if(pulse[i].active) return 0;
        for(uint8_t i=lo;i<hi;i++) {axes[i].state=0; axes[i].fault="NONE";
            axes[i].fault_axis=0; axes[i].fault_distance=0; axes[i].fault_target=0;}
        return 1;
    }
    if(c->type==CLAMP_GRIP || c->type==CLAMP_RELEASE) {
        for(uint8_t i=lo;i<hi;i++) if(!ready(&axes[i])) return 0;
        for(uint8_t i=lo;i<hi;i++) {
            ClampAxis *a=&axes[i]; a->target=c->type==CLAMP_GRIP?a->end:a->start;
            a->expected_sign=a->target>=a->distance?1:-1;
            a->motion_tick=a->progress_tick=HAL_GetTick(); a->progress_distance=a->distance;
            a->state=2;
        }
        return 1;
    }
    if(!c->axis) return 0;
    uint8_t i=c->axis-1; ClampAxis *a=&axes[i];
    if(a->state!=0) return 0;
    switch(c->type) {
    case CLAMP_UNTEACH: a->endpoints=0; a->calibrated=0; return 1;
    case CLAMP_JOG:
        if(c->value<1 || c->speed<100 || c->speed>CLAMP_MAX_PULSE_HZ || (c->sign!=0 && c->sign!=1)) return 0;
        if(a->endpoints==3) return 0; /* Endpoint capture locks out unbounded teaching jogs. */
        a->state=1; a->motion_tick=HAL_GetTick(); start_pulse(i,(uint8_t)c->sign,c->value,c->speed); return 1;
    case CLAMP_CALIBRATE:
        if(c->value>1 || (c->sign!=1 && c->sign!=-1)) return 0;
        a->close_level=(uint8_t)c->value; a->close_sign=(int8_t)c->sign; a->calibrated=1; return 1;
    case CLAMP_TEACH_START: case CLAMP_TEACH_END:
        if(!fresh(a)) return 0;
        if(c->type==CLAMP_TEACH_START) {a->start=a->distance; a->endpoints|=1;}
        else {a->end=a->distance; a->endpoints|=2;}
        endpoint_calibration(a); return 1;
    case CLAMP_SET_START: case CLAMP_SET_END:
        return set_endpoint(a,c->type==CLAMP_SET_END,c->value);
    case CLAMP_TOLERANCE:
        if(c->value<1 || c->value>100) return 0;
        a->tolerance=c->value; return 1;
    default: return 0;
    }
}
void Clamp_Task(void)
{
    uint32_t now=HAL_GetTick();
    for(uint8_t i=0;i<2;i++) {
        ClampAxis *a=&axes[i];
        if(a->state!=1 && a->state!=2) continue;
        if(a->state==2 && !fresh(a)) {Clamp_StopAll("STALE"); return;}
        /* Finite angle jogs finish by pulse count, including moves over 30s. */
        if(a->state==2 && now-a->motion_tick>MOVE_TIMEOUT_MS) {Clamp_StopAll("MOVE_TIMEOUT"); return;}
        uint32_t k=lock();
        uint8_t expired=pulse[i].expired;
        pulse[i].lease=TICK_HZ/10; unlock(k);
        if(expired) {Clamp_StopAll("LOOP_STALL"); return;}
        if(a->state==1) {if(!pulse[i].active) a->state=0; continue;}
        int64_t rem=((int64_t)a->target-a->distance)*a->expected_sign;
        /* One-shot positioning: reaching the tolerance band latches idle.
         * Later sensor noise cannot restart pulses; only a new motion command
         * can enter state 2 again (normally RELEASE after GRIP). */
        if(rem<=a->tolerance) {
            stop_pulse(i);
            a->state=0; /* Latch immediately; later noise cannot restart motion. */
            continue;
        }
        if(((int64_t)a->distance-a->progress_distance)*a->expected_sign>=1) {
            a->progress_tick=now; a->progress_distance=a->distance;
        } else if(now-a->progress_tick>30000U) {motion_fault(i,"NO_PROGRESS"); return;}
        if(!pulse[i].active) {
            uint8_t dir=a->expected_sign==a->close_sign?a->close_level:!a->close_level;
            start_pulse(i,dir,0,CLAMP_CRUISE_HZ);
        }
    }
}
