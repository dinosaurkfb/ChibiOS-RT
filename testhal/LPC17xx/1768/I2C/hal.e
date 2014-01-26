# 1 "../../../os/hal/src/hal.c"
# 1 "E:\\Dpan\\my_docs\\GCloud\\dev\\ChibiOS-RT\\testhal\\LPC17xx\\1768//"
# 1 "<command-line>"
# 1 "../../../os/hal/src/hal.c"
# 36 "../../../os/hal/src/hal.c"
# 1 "../../../os/kernel/include/ch.h" 1
# 110 "../../../os/kernel/include/ch.h"
# 1 "./chconf.h" 1
# 111 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/ports/GCC/ARMCMx/chtypes.h" 1
# 39 "../../../os/ports/GCC/ARMCMx/chtypes.h"
# 1 "e:\\yagarto-20121222\\bin\\../lib/gcc/arm-none-eabi/4.7.2/include/stddef.h" 1 3 4
# 150 "e:\\yagarto-20121222\\bin\\../lib/gcc/arm-none-eabi/4.7.2/include/stddef.h" 3 4
typedef int ptrdiff_t;
# 213 "e:\\yagarto-20121222\\bin\\../lib/gcc/arm-none-eabi/4.7.2/include/stddef.h" 3 4
typedef unsigned int size_t;
# 325 "e:\\yagarto-20121222\\bin\\../lib/gcc/arm-none-eabi/4.7.2/include/stddef.h" 3 4
typedef unsigned int wchar_t;
# 40 "../../../os/ports/GCC/ARMCMx/chtypes.h" 2
# 1 "e:\\yagarto-20121222\\bin\\../lib/gcc/arm-none-eabi/4.7.2/include/stdint.h" 1 3 4


# 1 "e:/yagarto-20121222/lib/gcc/../../arm-none-eabi/sys-include/stdint.h" 1 3 4
# 41 "e:/yagarto-20121222/lib/gcc/../../arm-none-eabi/sys-include/stdint.h" 3 4
typedef signed char int8_t ;
typedef unsigned char uint8_t ;




typedef signed char int_least8_t;
typedef unsigned char uint_least8_t;




typedef signed short int16_t;
typedef unsigned short uint16_t;
# 67 "e:/yagarto-20121222/lib/gcc/../../arm-none-eabi/sys-include/stdint.h" 3 4
typedef int16_t int_least16_t;
typedef uint16_t uint_least16_t;
# 79 "e:/yagarto-20121222/lib/gcc/../../arm-none-eabi/sys-include/stdint.h" 3 4
typedef signed long int32_t;
typedef unsigned long uint32_t;
# 97 "e:/yagarto-20121222/lib/gcc/../../arm-none-eabi/sys-include/stdint.h" 3 4
typedef int32_t int_least32_t;
typedef uint32_t uint_least32_t;
# 119 "e:/yagarto-20121222/lib/gcc/../../arm-none-eabi/sys-include/stdint.h" 3 4
typedef signed long long int64_t;
typedef unsigned long long uint64_t;
# 129 "e:/yagarto-20121222/lib/gcc/../../arm-none-eabi/sys-include/stdint.h" 3 4
typedef int64_t int_least64_t;
typedef uint64_t uint_least64_t;
# 159 "e:/yagarto-20121222/lib/gcc/../../arm-none-eabi/sys-include/stdint.h" 3 4
  typedef signed int int_fast8_t;
  typedef unsigned int uint_fast8_t;




  typedef signed int int_fast16_t;
  typedef unsigned int uint_fast16_t;




  typedef signed int int_fast32_t;
  typedef unsigned int uint_fast32_t;
# 213 "e:/yagarto-20121222/lib/gcc/../../arm-none-eabi/sys-include/stdint.h" 3 4
  typedef int_least64_t int_fast64_t;
  typedef uint_least64_t uint_fast64_t;







  typedef long long int intmax_t;
# 231 "e:/yagarto-20121222/lib/gcc/../../arm-none-eabi/sys-include/stdint.h" 3 4
  typedef long long unsigned int uintmax_t;
# 243 "e:/yagarto-20121222/lib/gcc/../../arm-none-eabi/sys-include/stdint.h" 3 4
typedef signed int intptr_t;
typedef unsigned int uintptr_t;
# 4 "e:\\yagarto-20121222\\bin\\../lib/gcc/arm-none-eabi/4.7.2/include/stdint.h" 2 3 4
# 41 "../../../os/ports/GCC/ARMCMx/chtypes.h" 2
# 1 "e:\\yagarto-20121222\\bin\\../lib/gcc/arm-none-eabi/4.7.2/include/stdbool.h" 1 3 4
# 42 "../../../os/ports/GCC/ARMCMx/chtypes.h" 2

typedef _Bool bool_t;
typedef uint8_t tmode_t;
typedef uint8_t tstate_t;
typedef uint8_t trefs_t;
typedef uint8_t tslices_t;
typedef uint32_t tprio_t;
typedef int32_t msg_t;
typedef int32_t eventid_t;
typedef uint32_t eventmask_t;
typedef uint32_t flagsmask_t;
typedef uint32_t systime_t;
typedef int32_t cnt_t;
# 112 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chlists.h" 1
# 42 "../../../os/kernel/include/chlists.h"
typedef struct Thread Thread;
# 97 "../../../os/kernel/include/chlists.h"
typedef struct {
  Thread *p_next;

  Thread *p_prev;

} ThreadsQueue;




typedef struct {

  Thread *p_next;


} ThreadsList;
# 113 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/ports/GCC/ARMCMx/chcore.h" 1
# 57 "../../../os/ports/GCC/ARMCMx/chcore.h"
# 1 "../../../os/ports/GCC/ARMCMx/LPC17xx/cmparams.h" 1
# 58 "../../../os/ports/GCC/ARMCMx/chcore.h" 2
# 138 "../../../os/ports/GCC/ARMCMx/chcore.h"
# 1 "../../../os/ports/GCC/ARMCMx/chcore_v7m.h" 1
# 229 "../../../os/ports/GCC/ARMCMx/chcore_v7m.h"
typedef void *regarm_t;





typedef uint64_t stkalign_t __attribute__ ((aligned (8)));

struct extctx {
  regarm_t r0;
  regarm_t r1;
  regarm_t r2;
  regarm_t r3;
  regarm_t r12;
  regarm_t lr_thd;
  regarm_t pc;
  regarm_t xpsr;
# 266 "../../../os/ports/GCC/ARMCMx/chcore_v7m.h"
};

struct intctx {
# 287 "../../../os/ports/GCC/ARMCMx/chcore_v7m.h"
  regarm_t r4;
  regarm_t r5;
  regarm_t r6;
  regarm_t r7;
  regarm_t r8;
  regarm_t r9;
  regarm_t r10;
  regarm_t r11;
  regarm_t lr;
};
# 305 "../../../os/ports/GCC/ARMCMx/chcore_v7m.h"
struct context {
  struct intctx *r13;
};
# 513 "../../../os/ports/GCC/ARMCMx/chcore_v7m.h"
  void port_halt(void);
  void _port_init(void);
  void _port_irq_epilogue(void);
  void _port_switch_from_isr(void);
  void _port_exit_from_isr(void);
  void _port_switch(Thread *ntp, Thread *otp);
  void _port_thread_start(void);
# 139 "../../../os/ports/GCC/ARMCMx/chcore.h" 2




# 1 "../../../os/ports/common/ARMCMx/nvic.h" 1
# 57 "../../../os/ports/common/ARMCMx/nvic.h"
typedef volatile uint8_t IOREG8;
typedef volatile uint32_t IOREG32;
# 68 "../../../os/ports/common/ARMCMx/nvic.h"
typedef struct {
  IOREG32 CSR;
  IOREG32 RVR;
  IOREG32 CVR;
  IOREG32 CBVR;
} CMx_ST;
# 106 "../../../os/ports/common/ARMCMx/nvic.h"
typedef struct {
  IOREG32 ISER[8];
  IOREG32 unused1[24];
  IOREG32 ICER[8];
  IOREG32 unused2[24];
  IOREG32 ISPR[8];
  IOREG32 unused3[24];
  IOREG32 ICPR[8];
  IOREG32 unused4[24];
  IOREG32 IABR[8];
  IOREG32 unused5[56];
  IOREG32 IPR[60];
  IOREG32 unused6[644];
  IOREG32 STIR;
} CMx_NVIC;
# 137 "../../../os/ports/common/ARMCMx/nvic.h"
typedef struct {
  IOREG32 CPUID;
  IOREG32 ICSR;
  IOREG32 VTOR;
  IOREG32 AIRCR;
  IOREG32 SCR;
  IOREG32 CCR;
  IOREG32 SHPR[3];
  IOREG32 SHCSR;
  IOREG32 CFSR;
  IOREG32 HFSR;
  IOREG32 DFSR;
  IOREG32 MMFAR;
  IOREG32 BFAR;
  IOREG32 AFSR;
  IOREG32 PFR[2];
  IOREG32 DFR;
  IOREG32 ADR;
  IOREG32 MMFR[4];
  IOREG32 SAR[5];
  IOREG32 unused1[5];
  IOREG32 CPACR;
} CMx_SCB;
# 204 "../../../os/ports/common/ARMCMx/nvic.h"
typedef struct {
  IOREG32 unused1[1];
  IOREG32 FPCCR;
  IOREG32 FPCAR;
  IOREG32 FPDSCR;
  IOREG32 MVFR0;
  IOREG32 MVFR1;
} CMx_FPU;
# 241 "../../../os/ports/common/ARMCMx/nvic.h"
typedef struct {
  IOREG32 DHCSR;
  IOREG32 DCRSR;
  IOREG32 DCRDR;
  IOREG32 DEMCR;
} CMx_SCS;
# 262 "../../../os/ports/common/ARMCMx/nvic.h"
typedef struct {
  IOREG32 CTRL;
  IOREG32 CYCCNT;
  IOREG32 CPICNT;
  IOREG32 EXCCNT;
  IOREG32 SLEEPCNT;
  IOREG32 LSUCNT;
  IOREG32 FOLDCNT;
  IOREG32 PCSR;
} CMx_DWT;
# 291 "../../../os/ports/common/ARMCMx/nvic.h"
  void nvicEnableVector(uint32_t n, uint32_t prio);
  void nvicDisableVector(uint32_t n);
  void nvicSetSystemHandlerPriority(uint32_t handler, uint32_t prio);
# 144 "../../../os/ports/GCC/ARMCMx/chcore.h" 2
# 114 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chsys.h" 1
# 246 "../../../os/kernel/include/chsys.h"
  void chSysInit(void);
  void chSysTimerHandlerI(void);
# 115 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chvt.h" 1
# 88 "../../../os/kernel/include/chvt.h"
typedef void (*vtfunc_t)(void *);




typedef struct VirtualTimer VirtualTimer;






struct VirtualTimer {
  VirtualTimer *vt_next;

  VirtualTimer *vt_prev;

  systime_t vt_time;
  vtfunc_t vt_func;

  void *vt_par;

};







typedef struct {
  VirtualTimer *vt_next;

  VirtualTimer *vt_prev;

  systime_t vt_time;
  volatile systime_t vt_systime;
} VTList;
# 245 "../../../os/kernel/include/chvt.h"
extern VTList vtlist;







  void _vt_init(void);
  void chVTSetI(VirtualTimer *vtp, systime_t time, vtfunc_t vtfunc, void *par);
  void chVTResetI(VirtualTimer *vtp);
# 116 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chschd.h" 1
# 94 "../../../os/kernel/include/chschd.h"
typedef struct {
  ThreadsQueue r_queue;
  tprio_t r_prio;

  struct context r_ctx;


  Thread *r_newer;
  Thread *r_older;


  Thread *r_current;

} ReadyList;



extern ReadyList rlist;
# 142 "../../../os/kernel/include/chschd.h"
  void _scheduler_init(void);

  Thread *chSchReadyI(Thread *tp);


  void chSchGoSleepS(tstate_t newstate);


  msg_t chSchGoSleepTimeoutS(tstate_t newstate, systime_t time);


  void chSchWakeupS(Thread *tp, msg_t msg);


  void chSchRescheduleS(void);





  void chSchDoRescheduleBehind(void);


  void chSchDoRescheduleAhead(void);


  void chSchDoReschedule(void);
# 117 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chsem.h" 1
# 44 "../../../os/kernel/include/chsem.h"
typedef struct Semaphore {
  ThreadsQueue s_queue;

  cnt_t s_cnt;
} Semaphore;




  void chSemInit(Semaphore *sp, cnt_t n);
  void chSemReset(Semaphore *sp, cnt_t n);
  void chSemResetI(Semaphore *sp, cnt_t n);
  msg_t chSemWait(Semaphore *sp);
  msg_t chSemWaitS(Semaphore *sp);
  msg_t chSemWaitTimeout(Semaphore *sp, systime_t time);
  msg_t chSemWaitTimeoutS(Semaphore *sp, systime_t time);
  void chSemSignal(Semaphore *sp);
  void chSemSignalI(Semaphore *sp);
  void chSemAddCounterI(Semaphore *sp, cnt_t n);

  msg_t chSemSignalWait(Semaphore *sps, Semaphore *spw);
# 118 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chbsem.h" 1
# 68 "../../../os/kernel/include/chbsem.h"
typedef struct {
  Semaphore bs_sem;
} BinarySemaphore;
# 119 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chmtx.h" 1
# 44 "../../../os/kernel/include/chmtx.h"
typedef struct Mutex {
  ThreadsQueue m_queue;

  Thread *m_owner;

  struct Mutex *m_next;

} Mutex;




  void chMtxInit(Mutex *mp);
  void chMtxLock(Mutex *mp);
  void chMtxLockS(Mutex *mp);
  bool_t chMtxTryLock(Mutex *mp);
  bool_t chMtxTryLockS(Mutex *mp);
  Mutex *chMtxUnlock(void);
  Mutex *chMtxUnlockS(void);
  void chMtxUnlockAll(void);
# 120 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chcond.h" 1
# 54 "../../../os/kernel/include/chcond.h"
typedef struct CondVar {
  ThreadsQueue c_queue;
} CondVar;




  void chCondInit(CondVar *cp);
  void chCondSignal(CondVar *cp);
  void chCondSignalI(CondVar *cp);
  void chCondBroadcast(CondVar *cp);
  void chCondBroadcastI(CondVar *cp);
  msg_t chCondWait(CondVar *cp);
  msg_t chCondWaitS(CondVar *cp);

  msg_t chCondWaitTimeout(CondVar *cp, systime_t time);
  msg_t chCondWaitTimeoutS(CondVar *cp, systime_t time);
# 121 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chevents.h" 1
# 44 "../../../os/kernel/include/chevents.h"
typedef struct EventListener EventListener;




struct EventListener {
  EventListener *el_next;


  Thread *el_listener;

  eventmask_t el_mask;


  flagsmask_t el_flags;

};




typedef struct EventSource {
  EventListener *es_next;


} EventSource;




typedef void (*evhandler_t)(eventid_t);
# 175 "../../../os/kernel/include/chevents.h"
  void chEvtRegisterMask(EventSource *esp,
                         EventListener *elp,
                         eventmask_t mask);
  void chEvtUnregister(EventSource *esp, EventListener *elp);
  eventmask_t chEvtGetAndClearEvents(eventmask_t mask);
  eventmask_t chEvtAddEvents(eventmask_t mask);
  flagsmask_t chEvtGetAndClearFlags(EventListener *elp);
  flagsmask_t chEvtGetAndClearFlagsI(EventListener *elp);
  void chEvtSignal(Thread *tp, eventmask_t mask);
  void chEvtSignalI(Thread *tp, eventmask_t mask);
  void chEvtBroadcastFlags(EventSource *esp, flagsmask_t flags);
  void chEvtBroadcastFlagsI(EventSource *esp, flagsmask_t flags);
  void chEvtDispatch(const evhandler_t *handlers, eventmask_t mask);

  eventmask_t chEvtWaitOne(eventmask_t mask);
  eventmask_t chEvtWaitAny(eventmask_t mask);
  eventmask_t chEvtWaitAll(eventmask_t mask);


  eventmask_t chEvtWaitOneTimeout(eventmask_t mask, systime_t time);
  eventmask_t chEvtWaitAnyTimeout(eventmask_t mask, systime_t time);
  eventmask_t chEvtWaitAllTimeout(eventmask_t mask, systime_t time);
# 122 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chmsg.h" 1
# 81 "../../../os/kernel/include/chmsg.h"
  msg_t chMsgSend(Thread *tp, msg_t msg);
  Thread * chMsgWait(void);
  void chMsgRelease(Thread *tp, msg_t msg);
# 123 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chmboxes.h" 1
# 51 "../../../os/kernel/include/chmboxes.h"
typedef struct {
  msg_t *mb_buffer;

  msg_t *mb_top;

  msg_t *mb_wrptr;
  msg_t *mb_rdptr;
  Semaphore mb_fullsem;

  Semaphore mb_emptysem;

} Mailbox;




  void chMBInit(Mailbox *mbp, msg_t *buf, cnt_t n);
  void chMBReset(Mailbox *mbp);
  msg_t chMBPost(Mailbox *mbp, msg_t msg, systime_t timeout);
  msg_t chMBPostS(Mailbox *mbp, msg_t msg, systime_t timeout);
  msg_t chMBPostI(Mailbox *mbp, msg_t msg);
  msg_t chMBPostAhead(Mailbox *mbp, msg_t msg, systime_t timeout);
  msg_t chMBPostAheadS(Mailbox *mbp, msg_t msg, systime_t timeout);
  msg_t chMBPostAheadI(Mailbox *mbp, msg_t msg);
  msg_t chMBFetch(Mailbox *mbp, msg_t *msgp, systime_t timeout);
  msg_t chMBFetchS(Mailbox *mbp, msg_t *msgp, systime_t timeout);
  msg_t chMBFetchI(Mailbox *mbp, msg_t *msgp);
# 124 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chmemcore.h" 1
# 44 "../../../os/kernel/include/chmemcore.h"
typedef void *(*memgetfunc_t)(size_t size);
# 81 "../../../os/kernel/include/chmemcore.h"
  void _core_init(void);
  void *chCoreAlloc(size_t size);
  void *chCoreAllocI(size_t size);
  size_t chCoreStatus(void);
# 125 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chheap.h" 1
# 52 "../../../os/kernel/include/chheap.h"
typedef struct memory_heap MemoryHeap;




union heap_header {
  stkalign_t align;
  struct {
    union {
      union heap_header *next;
      MemoryHeap *heap;
    } u;
    size_t size;
  } h;
};




struct memory_heap {
  memgetfunc_t h_provider;

  union heap_header h_free;

  Mutex h_mtx;



};




  void _heap_init(void);

  void chHeapInit(MemoryHeap *heapp, void *buf, size_t size);

  void *chHeapAlloc(MemoryHeap *heapp, size_t size);
  void chHeapFree(void *p);
  size_t chHeapStatus(MemoryHeap *heapp, size_t *sizep);
# 126 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chmempools.h" 1
# 44 "../../../os/kernel/include/chmempools.h"
struct pool_header {
  struct pool_header *ph_next;

};




typedef struct {
  struct pool_header *mp_next;
  size_t mp_object_size;

  memgetfunc_t mp_provider;

} MemoryPool;
# 127 "../../../os/kernel/include/chmempools.h"
  void chPoolInit(MemoryPool *mp, size_t size, memgetfunc_t provider);
  void chPoolLoadArray(MemoryPool *mp, void *p, size_t n);
  void *chPoolAllocI(MemoryPool *mp);
  void *chPoolAlloc(MemoryPool *mp);
  void chPoolFreeI(MemoryPool *mp, void *objp);
  void chPoolFree(MemoryPool *mp, void *objp);
# 127 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chthreads.h" 1
# 94 "../../../os/kernel/include/chthreads.h"
struct Thread {
  Thread *p_next;

  Thread *p_prev;

  tprio_t p_prio;
  struct context p_ctx;

  Thread *p_newer;
  Thread *p_older;






  const char *p_name;
# 121 "../../../os/kernel/include/chthreads.h"
  tstate_t p_state;



  tmode_t p_flags;




  trefs_t p_refs;





  tslices_t p_preempt;






  volatile systime_t p_time;






  union {






    msg_t rdymsg;






    msg_t exitcode;






    void *wtobjp;






    eventmask_t ewmask;

  } p_u;




  ThreadsList p_waiting;





  ThreadsQueue p_msgqueue;



  msg_t p_msg;





  eventmask_t p_epending;






  Mutex *p_mtxlist;



  tprio_t p_realprio;





  void *p_mpool;



 

};




typedef msg_t (*tfunc_t)(void *);
# 362 "../../../os/kernel/include/chthreads.h"
  Thread *_thread_init(Thread *tp, tprio_t prio);



  Thread *chThdCreateI(void *wsp, size_t size,
                       tprio_t prio, tfunc_t pf, void *arg);
  Thread *chThdCreateStatic(void *wsp, size_t size,
                            tprio_t prio, tfunc_t pf, void *arg);
  tprio_t chThdSetPriority(tprio_t newprio);
  Thread *chThdResume(Thread *tp);
  void chThdTerminate(Thread *tp);
  void chThdSleep(systime_t time);
  void chThdSleepUntil(systime_t time);
  void chThdYield(void);
  void chThdExit(msg_t msg);
  void chThdExitS(msg_t msg);

  msg_t chThdWait(Thread *tp);
# 128 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chdynamic.h" 1
# 57 "../../../os/kernel/include/chdynamic.h"
  Thread *chThdAddRef(Thread *tp);
  void chThdRelease(Thread *tp);

  Thread *chThdCreateFromHeap(MemoryHeap *heapp, size_t size,
                              tprio_t prio, tfunc_t pf, void *arg);


  Thread *chThdCreateFromMemoryPool(MemoryPool *mp, tprio_t prio,
                                    tfunc_t pf, void *arg);
# 129 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chregistry.h" 1
# 44 "../../../os/kernel/include/chregistry.h"
typedef struct {
  char ch_identifier[4];
  uint8_t ch_zero;
  uint8_t ch_size;
  uint16_t ch_version;
  uint8_t ch_ptrsize;
  uint8_t ch_timesize;
  uint8_t ch_threadsize;
  uint8_t cf_off_prio;
  uint8_t cf_off_ctx;
  uint8_t cf_off_newer;
  uint8_t cf_off_older;
  uint8_t cf_off_name;
  uint8_t cf_off_stklimit;

  uint8_t cf_off_state;
  uint8_t cf_off_flags;
  uint8_t cf_off_refs;
  uint8_t cf_off_preempt;

  uint8_t cf_off_time;
} chdebug_t;
# 126 "../../../os/kernel/include/chregistry.h"
  extern const chdebug_t ch_debug;
  Thread *chRegFirstThread(void);
  Thread *chRegNextThread(Thread *tp);
# 130 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chinline.h" 1
# 41 "../../../os/kernel/include/chinline.h"
static inline void prio_insert(Thread *tp, ThreadsQueue *tqp) {

  Thread *cp = (Thread *)tqp;
  do {
    cp = cp->p_next;
  } while ((cp != (Thread *)tqp) && (cp->p_prio >= tp->p_prio));
  tp->p_next = cp;
  tp->p_prev = cp->p_prev;
  tp->p_prev->p_next = cp->p_prev = tp;
}

static inline void queue_insert(Thread *tp, ThreadsQueue *tqp) {

  tp->p_next = (Thread *)tqp;
  tp->p_prev = tqp->p_prev;
  tp->p_prev->p_next = tqp->p_prev = tp;
}

static inline Thread *fifo_remove(ThreadsQueue *tqp) {
  Thread *tp = tqp->p_next;

  (tqp->p_next = tp->p_next)->p_prev = (Thread *)tqp;
  return tp;
}

static inline Thread *lifo_remove(ThreadsQueue *tqp) {
  Thread *tp = tqp->p_prev;

  (tqp->p_prev = tp->p_prev)->p_next = (Thread *)tqp;
  return tp;
}

static inline Thread *dequeue(Thread *tp) {

  tp->p_prev->p_next = tp->p_next;
  tp->p_next->p_prev = tp->p_prev;
  return tp;
}

static inline void list_insert(Thread *tp, ThreadsList *tlp) {

  tp->p_next = tlp->p_next;
  tlp->p_next = tp;
}

static inline Thread *list_remove(ThreadsList *tlp) {

  Thread *tp = tlp->p_next;
  tlp->p_next = tp->p_next;
  return tp;
}
# 131 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chqueues.h" 1
# 55 "../../../os/kernel/include/chqueues.h"
typedef struct GenericQueue GenericQueue;


typedef void (*qnotify_t)(GenericQueue *qp);
# 69 "../../../os/kernel/include/chqueues.h"
struct GenericQueue {
  ThreadsQueue q_waiting;
  size_t q_counter;
  uint8_t *q_buffer;
  uint8_t *q_top;

  uint8_t *q_wrptr;
  uint8_t *q_rdptr;
  qnotify_t q_notify;
  void *q_link;
};
# 130 "../../../os/kernel/include/chqueues.h"
typedef GenericQueue InputQueue;
# 245 "../../../os/kernel/include/chqueues.h"
typedef GenericQueue OutputQueue;
# 354 "../../../os/kernel/include/chqueues.h"
  void chIQInit(InputQueue *iqp, uint8_t *bp, size_t size, qnotify_t infy,
                void *link);
  void chIQResetI(InputQueue *iqp);
  msg_t chIQPutI(InputQueue *iqp, uint8_t b);
  msg_t chIQGetTimeout(InputQueue *iqp, systime_t time);
  size_t chIQReadTimeout(InputQueue *iqp, uint8_t *bp,
                         size_t n, systime_t time);

  void chOQInit(OutputQueue *oqp, uint8_t *bp, size_t size, qnotify_t onfy,
                void *link);
  void chOQResetI(OutputQueue *oqp);
  msg_t chOQPutTimeout(OutputQueue *oqp, uint8_t b, systime_t time);
  msg_t chOQGetI(OutputQueue *oqp);
  size_t chOQWriteTimeout(OutputQueue *oqp, const uint8_t *bp,
                          size_t n, systime_t time);
# 132 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chstreams.h" 1
# 72 "../../../os/kernel/include/chstreams.h"
struct BaseSequentialStreamVMT {
  size_t (*write)(void *instance, const uint8_t *bp, size_t n); size_t (*read)(void *instance, uint8_t *bp, size_t n); msg_t (*put)(void *instance, uint8_t b); msg_t (*get)(void *instance);
};






typedef struct {

  const struct BaseSequentialStreamVMT *vmt;
 
} BaseSequentialStream;
# 133 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chfiles.h" 1
# 62 "../../../os/kernel/include/chfiles.h"
typedef uint32_t fileoffset_t;
# 93 "../../../os/kernel/include/chfiles.h"
struct BaseFileStreamVMT {
  size_t (*write)(void *instance, const uint8_t *bp, size_t n); size_t (*read)(void *instance, uint8_t *bp, size_t n); msg_t (*put)(void *instance, uint8_t b); msg_t (*get)(void *instance); uint32_t (*close)(void *instance); int (*geterror)(void *instance); fileoffset_t (*getsize)(void *instance); fileoffset_t (*getposition)(void *instance); uint32_t (*lseek)(void *instance, fileoffset_t offset);
};







typedef struct {

  const struct BaseFileStreamVMT *vmt;
 
} BaseFileStream;
# 134 "../../../os/kernel/include/ch.h" 2
# 1 "../../../os/kernel/include/chdebug.h" 1
# 135 "../../../os/kernel/include/ch.h" 2


extern stkalign_t _idle_thread_wa[((((sizeof(Thread) + sizeof(struct intctx) + sizeof(struct extctx) + (16) + (32)) - 1) | (sizeof(stkalign_t) - 1)) + 1) / sizeof(stkalign_t)];





  void _idle_thread(void *p);
# 37 "../../../os/hal/src/hal.c" 2
# 1 "../../../os/hal/include/hal.h" 1
# 40 "../../../os/hal/include/hal.h"
# 1 "../../../boards/HY-LandTiger_1768/board.h" 1
# 98 "../../../boards/HY-LandTiger_1768/board.h"
  void boardInit(void);
  void msDelay (uint32_t ulTime);
  void ledSingleBlinkBin(uint32_t num, uint32_t interval);
  void ledDoubleBlinkBin(uint32_t num, uint32_t interval);
  void LOG_PRINT(const char *fmt, ...);
# 41 "../../../os/hal/include/hal.h" 2
# 1 "./halconf.h" 1
# 31 "./halconf.h"
# 1 "./mcuconf.h" 1
# 32 "./halconf.h" 2
# 42 "../../../os/hal/include/hal.h" 2

# 1 "../../../os/hal/platforms/LPC17xx/hal_lld.h" 1
# 29 "../../../os/hal/platforms/LPC17xx/hal_lld.h"
# 1 "../../../os/hal/platforms/LPC17xx/LPC17xx.h" 1
# 36 "../../../os/hal/platforms/LPC17xx/LPC17xx.h"
typedef enum IRQn
{

  NonMaskableInt_IRQn = -14,
  MemoryManagement_IRQn = -12,
  BusFault_IRQn = -11,
  UsageFault_IRQn = -10,
  SVCall_IRQn = -5,
  DebugMonitor_IRQn = -4,
  PendSV_IRQn = -2,
  SysTick_IRQn = -1,


  WDT_IRQn = 0,
  TIMER0_IRQn = 1,
  TIMER1_IRQn = 2,
  TIMER2_IRQn = 3,
  TIMER3_IRQn = 4,
  UART0_IRQn = 5,
  UART1_IRQn = 6,
  UART2_IRQn = 7,
  UART3_IRQn = 8,
  PWM1_IRQn = 9,
  I2C0_IRQn = 10,
  I2C1_IRQn = 11,
  I2C2_IRQn = 12,
  SPI_IRQn = 13,
  SSP0_IRQn = 14,
  SSP1_IRQn = 15,
  PLL0_IRQn = 16,
  RTC_IRQn = 17,
  EINT0_IRQn = 18,
  EINT1_IRQn = 19,
  EINT2_IRQn = 20,
  EINT3_IRQn = 21,
  ADC_IRQn = 22,
  BOD_IRQn = 23,
  USB_IRQn = 24,
  CAN_IRQn = 25,
  DMA_IRQn = 26,
  I2S_IRQn = 27,
  ENET_IRQn = 28,
  RIT_IRQn = 29,
  MCPWM_IRQn = 30,
  QEI_IRQn = 31,
  PLL1_IRQn = 32,
  USBActivity_IRQn = 33,
  CANActivity_IRQn = 34,
} IRQn_Type;
# 99 "../../../os/hal/platforms/LPC17xx/LPC17xx.h"
# 1 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h" 1
# 121 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
# 1 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmInstr.h" 1
# 286 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline void __NOP(void)
{
  __asm volatile ("nop");
}







__attribute__( ( always_inline ) ) static inline void __WFI(void)
{
  __asm volatile ("wfi");
}







__attribute__( ( always_inline ) ) static inline void __WFE(void)
{
  __asm volatile ("wfe");
}






__attribute__( ( always_inline ) ) static inline void __SEV(void)
{
  __asm volatile ("sev");
}
# 330 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline void __ISB(void)
{
  __asm volatile ("isb");
}







__attribute__( ( always_inline ) ) static inline void __DSB(void)
{
  __asm volatile ("dsb");
}







__attribute__( ( always_inline ) ) static inline void __DMB(void)
{
  __asm volatile ("dmb");
}
# 365 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __REV(uint32_t value)
{
  uint32_t result;

  __asm volatile ("rev %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}
# 381 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __REV16(uint32_t value)
{
  uint32_t result;

  __asm volatile ("rev16 %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}
# 397 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline int32_t __REVSH(int32_t value)
{
  uint32_t result;

  __asm volatile ("revsh %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}
# 414 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __ROR(uint32_t op1, uint32_t op2)
{

  __asm volatile ("ror %0, %0, %1" : "+r" (op1) : "r" (op2) );
  return(op1);
}
# 431 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __RBIT(uint32_t value)
{
  uint32_t result;

   __asm volatile ("rbit %0, %1" : "=r" (result) : "r" (value) );
   return(result);
}
# 447 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint8_t __LDREXB(volatile uint8_t *addr)
{
    uint8_t result;

   __asm volatile ("ldrexb %0, [%1]" : "=r" (result) : "r" (addr) );
   return(result);
}
# 463 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint16_t __LDREXH(volatile uint16_t *addr)
{
    uint16_t result;

   __asm volatile ("ldrexh %0, [%1]" : "=r" (result) : "r" (addr) );
   return(result);
}
# 479 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __LDREXW(volatile uint32_t *addr)
{
    uint32_t result;

   __asm volatile ("ldrex %0, [%1]" : "=r" (result) : "r" (addr) );
   return(result);
}
# 497 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __STREXB(uint8_t value, volatile uint8_t *addr)
{
   uint32_t result;

   __asm volatile ("strexb %0, %2, [%1]" : "=&r" (result) : "r" (addr), "r" (value) );
   return(result);
}
# 515 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __STREXH(uint16_t value, volatile uint16_t *addr)
{
   uint32_t result;

   __asm volatile ("strexh %0, %2, [%1]" : "=&r" (result) : "r" (addr), "r" (value) );
   return(result);
}
# 533 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __STREXW(uint32_t value, volatile uint32_t *addr)
{
   uint32_t result;

   __asm volatile ("strex %0, %2, [%1]" : "=&r" (result) : "r" (addr), "r" (value) );
   return(result);
}







__attribute__( ( always_inline ) ) static inline void __CLREX(void)
{
  __asm volatile ("clrex");
}
# 592 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint8_t __CLZ(uint32_t value)
{
  uint8_t result;

  __asm volatile ("clz %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}
# 122 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h" 2
# 1 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h" 1
# 315 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __enable_irq(void)
{
  __asm volatile ("cpsie i");
}







__attribute__( ( always_inline ) ) static inline void __disable_irq(void)
{
  __asm volatile ("cpsid i");
}
# 338 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_CONTROL(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, control" : "=r" (result) );
  return(result);
}
# 353 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_CONTROL(uint32_t control)
{
  __asm volatile ("MSR control, %0" : : "r" (control) );
}
# 365 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_IPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, ipsr" : "=r" (result) );
  return(result);
}
# 380 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_APSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, apsr" : "=r" (result) );
  return(result);
}
# 395 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_xPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, xpsr" : "=r" (result) );
  return(result);
}
# 410 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_PSP(void)
{
  register uint32_t result;

  __asm volatile ("MRS %0, psp\n" : "=r" (result) );
  return(result);
}
# 425 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_PSP(uint32_t topOfProcStack)
{
  __asm volatile ("MSR psp, %0\n" : : "r" (topOfProcStack) );
}
# 437 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_MSP(void)
{
  register uint32_t result;

  __asm volatile ("MRS %0, msp\n" : "=r" (result) );
  return(result);
}
# 452 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_MSP(uint32_t topOfMainStack)
{
  __asm volatile ("MSR msp, %0\n" : : "r" (topOfMainStack) );
}
# 464 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_PRIMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, primask" : "=r" (result) );
  return(result);
}
# 479 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_PRIMASK(uint32_t priMask)
{
  __asm volatile ("MSR primask, %0" : : "r" (priMask) );
}
# 492 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __enable_fault_irq(void)
{
  __asm volatile ("cpsie f");
}







__attribute__( ( always_inline ) ) static inline void __disable_fault_irq(void)
{
  __asm volatile ("cpsid f");
}
# 515 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_BASEPRI(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, basepri_max" : "=r" (result) );
  return(result);
}
# 530 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_BASEPRI(uint32_t value)
{
  __asm volatile ("MSR basepri, %0" : : "r" (value) );
}
# 542 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_FAULTMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, faultmask" : "=r" (result) );
  return(result);
}
# 557 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_FAULTMASK(uint32_t faultMask)
{
  __asm volatile ("MSR faultmask, %0" : : "r" (faultMask) );
}
# 123 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h" 2
# 196 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
typedef union
{
  struct
  {

    uint32_t _reserved0:27;





    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} APSR_Type;




typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:23;
  } b;
  uint32_t w;
} IPSR_Type;




typedef union
{
  struct
  {
    uint32_t ISR:9;

    uint32_t _reserved0:15;





    uint32_t T:1;
    uint32_t IT:2;
    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} xPSR_Type;




typedef union
{
  struct
  {
    uint32_t nPRIV:1;
    uint32_t SPSEL:1;
    uint32_t FPCA:1;
    uint32_t _reserved0:29;
  } b;
  uint32_t w;
} CONTROL_Type;
# 281 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
typedef struct
{
  volatile uint32_t ISER[8];
       uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];
       uint32_t RSERVED1[24];
  volatile uint32_t ISPR[8];
       uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];
       uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];
       uint32_t RESERVED4[56];
  volatile uint8_t IP[240];
       uint32_t RESERVED5[644];
  volatile uint32_t STIR;
} NVIC_Type;
# 313 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
typedef struct
{
  volatile const uint32_t CPUID;
  volatile uint32_t ICSR;
  volatile uint32_t VTOR;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
  volatile uint8_t SHP[12];
  volatile uint32_t SHCSR;
  volatile uint32_t CFSR;
  volatile uint32_t HFSR;
  volatile uint32_t DFSR;
  volatile uint32_t MMFAR;
  volatile uint32_t BFAR;
  volatile uint32_t AFSR;
  volatile const uint32_t PFR[2];
  volatile const uint32_t DFR;
  volatile const uint32_t ADR;
  volatile const uint32_t MMFR[4];
  volatile const uint32_t ISAR[5];
       uint32_t RESERVED0[5];
  volatile uint32_t CPACR;
} SCB_Type;
# 538 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
typedef struct
{
       uint32_t RESERVED0[1];
  volatile const uint32_t ICTR;



       uint32_t RESERVED1[1];

} SCnSCB_Type;
# 575 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile const uint32_t CALIB;
} SysTick_Type;
# 625 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
typedef struct
{
  volatile union
  {
    volatile uint8_t u8;
    volatile uint16_t u16;
    volatile uint32_t u32;
  } PORT [32];
       uint32_t RESERVED0[864];
  volatile uint32_t TER;
       uint32_t RESERVED1[15];
  volatile uint32_t TPR;
       uint32_t RESERVED2[15];
  volatile uint32_t TCR;
       uint32_t RESERVED3[29];
  volatile uint32_t IWR;
  volatile const uint32_t IRR;
  volatile uint32_t IMCR;
       uint32_t RESERVED4[43];
  volatile uint32_t LAR;
  volatile const uint32_t LSR;
       uint32_t RESERVED5[6];
  volatile const uint32_t PID4;
  volatile const uint32_t PID5;
  volatile const uint32_t PID6;
  volatile const uint32_t PID7;
  volatile const uint32_t PID0;
  volatile const uint32_t PID1;
  volatile const uint32_t PID2;
  volatile const uint32_t PID3;
  volatile const uint32_t CID0;
  volatile const uint32_t CID1;
  volatile const uint32_t CID2;
  volatile const uint32_t CID3;
} ITM_Type;
# 726 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t CYCCNT;
  volatile uint32_t CPICNT;
  volatile uint32_t EXCCNT;
  volatile uint32_t SLEEPCNT;
  volatile uint32_t LSUCNT;
  volatile uint32_t FOLDCNT;
  volatile const uint32_t PCSR;
  volatile uint32_t COMP0;
  volatile uint32_t MASK0;
  volatile uint32_t FUNCTION0;
       uint32_t RESERVED0[1];
  volatile uint32_t COMP1;
  volatile uint32_t MASK1;
  volatile uint32_t FUNCTION1;
       uint32_t RESERVED1[1];
  volatile uint32_t COMP2;
  volatile uint32_t MASK2;
  volatile uint32_t FUNCTION2;
       uint32_t RESERVED2[1];
  volatile uint32_t COMP3;
  volatile uint32_t MASK3;
  volatile uint32_t FUNCTION3;
} DWT_Type;
# 871 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
typedef struct
{
  volatile uint32_t SSPSR;
  volatile uint32_t CSPSR;
       uint32_t RESERVED0[2];
  volatile uint32_t ACPR;
       uint32_t RESERVED1[55];
  volatile uint32_t SPPR;
       uint32_t RESERVED2[131];
  volatile const uint32_t FFSR;
  volatile uint32_t FFCR;
  volatile const uint32_t FSCR;
       uint32_t RESERVED3[759];
  volatile const uint32_t TRIGGER;
  volatile const uint32_t FIFO0;
  volatile const uint32_t ITATBCTR2;
       uint32_t RESERVED4[1];
  volatile const uint32_t ITATBCTR0;
  volatile const uint32_t FIFO1;
  volatile uint32_t ITCTRL;
       uint32_t RESERVED5[39];
  volatile uint32_t CLAIMSET;
  volatile uint32_t CLAIMCLR;
       uint32_t RESERVED7[8];
  volatile const uint32_t DEVID;
  volatile const uint32_t DEVTYPE;
} TPI_Type;
# 1025 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
typedef struct
{
  volatile const uint32_t TYPE;
  volatile uint32_t CTRL;
  volatile uint32_t RNR;
  volatile uint32_t RBAR;
  volatile uint32_t RASR;
  volatile uint32_t RBAR_A1;
  volatile uint32_t RASR_A1;
  volatile uint32_t RBAR_A2;
  volatile uint32_t RASR_A2;
  volatile uint32_t RBAR_A3;
  volatile uint32_t RASR_A3;
} MPU_Type;
# 1117 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
typedef struct
{
  volatile uint32_t DHCSR;
  volatile uint32_t DCRSR;
  volatile uint32_t DCRDR;
  volatile uint32_t DEMCR;
} CoreDebug_Type;
# 1276 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07);

  reg_value = ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;
  reg_value &= ~((0xFFFFUL << 16) | (7UL << 8));
  reg_value = (reg_value |
                ((uint32_t)0x5FA << 16) |
                (PriorityGroupTmp << 8));
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = reg_value;
}
# 1296 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) >> 8);
}
# 1308 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 1320 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 1336 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}
# 1348 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 1360 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 1375 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}
# 1390 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 5)) & 0xff); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)] = ((priority << (8 - 5)) & 0xff); }
}
# 1410 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 5))); }
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)] >> (8 - 5))); }
}
# 1432 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 5) ? 5 : 7 - PriorityGroupTmp;
  SubPriorityBits = ((PriorityGroupTmp + 5) < 7) ? 0 : PriorityGroupTmp - 7 + 5;

  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority & ((1 << (SubPriorityBits )) - 1)))
         );
}
# 1460 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 5) ? 5 : 7 - PriorityGroupTmp;
  SubPriorityBits = ((PriorityGroupTmp + 5) < 7) ? 0 : PriorityGroupTmp - 7 + 5;

  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority = (Priority ) & ((1 << (SubPriorityBits )) - 1);
}






static inline void NVIC_SystemReset(void)
{
  __DSB();

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = ((0x5FA << 16) |
                 (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) |
                 (1UL << 2));
  __DSB();
  while(1);
}
# 1517 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline uint32_t SysTick_Config(uint32_t ticks)
{
  if (ticks > (0xFFFFFFUL << 0)) return (1);

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = (ticks & (0xFFFFFFUL << 0)) - 1;
  NVIC_SetPriority (SysTick_IRQn, (1<<5) - 1);
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL = 0;
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2) |
                   (1UL << 1) |
                   (1UL << 0);
  return (0);
}
# 1543 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
extern volatile int32_t ITM_RxBuffer;
# 1557 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL << 0)) &&
      (((ITM_Type *) (0xE0000000UL) )->TER & (1UL << 0) ) )
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000UL) )->PORT[0].u8 = (uint8_t) ch;
  }
  return (ch);
}
# 1576 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline int32_t ITM_ReceiveChar (void) {
  int32_t ch = -1;

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;
  }

  return (ch);
}
# 1595 "../../../os/ports/common/ARMCMx/CMSIS/include/core_cm3.h"
static inline int32_t ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);
  } else {
    return (1);
  }
}
# 100 "../../../os/hal/platforms/LPC17xx/LPC17xx.h" 2
# 1 "../../../os/hal/platforms/LPC17xx/system_LPC17xx.h" 1
# 35 "../../../os/hal/platforms/LPC17xx/system_LPC17xx.h"
extern uint32_t SystemCoreClock;
# 47 "../../../os/hal/platforms/LPC17xx/system_LPC17xx.h"
extern void SystemInit (void);
# 58 "../../../os/hal/platforms/LPC17xx/system_LPC17xx.h"
extern void SystemCoreClockUpdate (void);
# 101 "../../../os/hal/platforms/LPC17xx/LPC17xx.h" 2
# 112 "../../../os/hal/platforms/LPC17xx/LPC17xx.h"
typedef struct
{
  volatile uint32_t FLASHCFG;
       uint32_t RESERVED0[31];
  volatile uint32_t PLL0CON;
  volatile uint32_t PLL0CFG;
  volatile const uint32_t PLL0STAT;
  volatile uint32_t PLL0FEED;
       uint32_t RESERVED1[4];
  volatile uint32_t PLL1CON;
  volatile uint32_t PLL1CFG;
  volatile const uint32_t PLL1STAT;
  volatile uint32_t PLL1FEED;
       uint32_t RESERVED2[4];
  volatile uint32_t PCON;
  volatile uint32_t PCONP;
       uint32_t RESERVED3[15];
  volatile uint32_t CCLKCFG;
  volatile uint32_t USBCLKCFG;
  volatile uint32_t CLKSRCSEL;
  volatile uint32_t CANSLEEPCLR;
  volatile uint32_t CANWAKEFLAGS;
       uint32_t RESERVED4[10];
  volatile uint32_t EXTINT;
       uint32_t RESERVED5;
  volatile uint32_t EXTMODE;
  volatile uint32_t EXTPOLAR;
       uint32_t RESERVED6[12];
  volatile uint32_t RSID;
       uint32_t RESERVED7[7];
  volatile uint32_t SCS;
  volatile uint32_t IRCTRIM;
  volatile uint32_t PCLKSEL0;
  volatile uint32_t PCLKSEL1;
       uint32_t RESERVED8[4];
  volatile uint32_t USBIntSt;
  volatile uint32_t DMAREQSEL;
  volatile uint32_t CLKOUTCFG;
 } LPC_SC_TypeDef;


typedef struct
{
  volatile uint32_t PINSEL0;
  volatile uint32_t PINSEL1;
  volatile uint32_t PINSEL2;
  volatile uint32_t PINSEL3;
  volatile uint32_t PINSEL4;
  volatile uint32_t PINSEL5;
  volatile uint32_t PINSEL6;
  volatile uint32_t PINSEL7;
  volatile uint32_t PINSEL8;
  volatile uint32_t PINSEL9;
  volatile uint32_t PINSEL10;
       uint32_t RESERVED0[5];
  volatile uint32_t PINMODE0;
  volatile uint32_t PINMODE1;
  volatile uint32_t PINMODE2;
  volatile uint32_t PINMODE3;
  volatile uint32_t PINMODE4;
  volatile uint32_t PINMODE5;
  volatile uint32_t PINMODE6;
  volatile uint32_t PINMODE7;
  volatile uint32_t PINMODE8;
  volatile uint32_t PINMODE9;
  volatile uint32_t PINMODE_OD0;
  volatile uint32_t PINMODE_OD1;
  volatile uint32_t PINMODE_OD2;
  volatile uint32_t PINMODE_OD3;
  volatile uint32_t PINMODE_OD4;
  volatile uint32_t I2CPADCFG;
} LPC_PINCON_TypeDef;


typedef struct
{
  union {
    volatile uint32_t FIODIR;
    struct {
      volatile uint16_t FIODIRL;
      volatile uint16_t FIODIRH;
    };
    struct {
      volatile uint8_t FIODIR0;
      volatile uint8_t FIODIR1;
      volatile uint8_t FIODIR2;
      volatile uint8_t FIODIR3;
    };
  };
  uint32_t RESERVED0[3];
  union {
    volatile uint32_t FIOMASK;
    struct {
      volatile uint16_t FIOMASKL;
      volatile uint16_t FIOMASKH;
    };
    struct {
      volatile uint8_t FIOMASK0;
      volatile uint8_t FIOMASK1;
      volatile uint8_t FIOMASK2;
      volatile uint8_t FIOMASK3;
    };
  };
  union {
    volatile uint32_t FIOPIN;
    struct {
      volatile uint16_t FIOPINL;
      volatile uint16_t FIOPINH;
    };
    struct {
      volatile uint8_t FIOPIN0;
      volatile uint8_t FIOPIN1;
      volatile uint8_t FIOPIN2;
      volatile uint8_t FIOPIN3;
    };
  };
  union {
    volatile uint32_t FIOSET;
    struct {
      volatile uint16_t FIOSETL;
      volatile uint16_t FIOSETH;
    };
    struct {
      volatile uint8_t FIOSET0;
      volatile uint8_t FIOSET1;
      volatile uint8_t FIOSET2;
      volatile uint8_t FIOSET3;
    };
  };
  union {
    volatile uint32_t FIOCLR;
    struct {
      volatile uint16_t FIOCLRL;
      volatile uint16_t FIOCLRH;
    };
    struct {
      volatile uint8_t FIOCLR0;
      volatile uint8_t FIOCLR1;
      volatile uint8_t FIOCLR2;
      volatile uint8_t FIOCLR3;
    };
  };
} LPC_GPIO_TypeDef;

typedef struct
{
  volatile const uint32_t IntStatus;
  volatile const uint32_t IO0IntStatR;
  volatile const uint32_t IO0IntStatF;
  volatile uint32_t IO0IntClr;
  volatile uint32_t IO0IntEnR;
  volatile uint32_t IO0IntEnF;
       uint32_t RESERVED0[3];
  volatile const uint32_t IO2IntStatR;
  volatile const uint32_t IO2IntStatF;
  volatile uint32_t IO2IntClr;
  volatile uint32_t IO2IntEnR;
  volatile uint32_t IO2IntEnF;
} LPC_GPIOINT_TypeDef;


typedef struct
{
  volatile uint32_t IR;
  volatile uint32_t TCR;
  volatile uint32_t TC;
  volatile uint32_t PR;
  volatile uint32_t PC;
  volatile uint32_t MCR;
  volatile uint32_t MR0;
  volatile uint32_t MR1;
  volatile uint32_t MR2;
  volatile uint32_t MR3;
  volatile uint32_t CCR;
  volatile const uint32_t CR0;
  volatile const uint32_t CR1;
       uint32_t RESERVED0[2];
  volatile uint32_t EMR;
       uint32_t RESERVED1[12];
  volatile uint32_t CTCR;
} LPC_TIM_TypeDef;


typedef struct
{
  volatile uint32_t IR;
  volatile uint32_t TCR;
  volatile uint32_t TC;
  volatile uint32_t PR;
  volatile uint32_t PC;
  volatile uint32_t MCR;
  volatile uint32_t MR0;
  volatile uint32_t MR1;
  volatile uint32_t MR2;
  volatile uint32_t MR3;
  volatile uint32_t CCR;
  volatile const uint32_t CR0;
  volatile const uint32_t CR1;
  volatile const uint32_t CR2;
  volatile const uint32_t CR3;
       uint32_t RESERVED0;
  volatile uint32_t MR4;
  volatile uint32_t MR5;
  volatile uint32_t MR6;
  volatile uint32_t PCR;
  volatile uint32_t LER;
       uint32_t RESERVED1[7];
  volatile uint32_t CTCR;
} LPC_PWM_TypeDef;


typedef struct
{
  union {
  volatile const uint8_t RBR;
  volatile uint8_t THR;
  volatile uint8_t DLL;
       uint32_t RESERVED0;
  };
  union {
  volatile uint8_t DLM;
  volatile uint32_t IER;
  };
  union {
  volatile const uint32_t IIR;
  volatile uint8_t FCR;
  };
  volatile uint8_t LCR;
       uint8_t RESERVED1[7];
  volatile const uint8_t LSR;
       uint8_t RESERVED2[7];
  volatile uint8_t SCR;
       uint8_t RESERVED3[3];
  volatile uint32_t ACR;
  volatile uint8_t ICR;
       uint8_t RESERVED4[3];
  volatile uint8_t FDR;
       uint8_t RESERVED5[7];
  volatile uint8_t TER;
       uint8_t RESERVED6[39];
  volatile uint32_t FIFOLVL;
} LPC_UART_TypeDef;

typedef struct
{
  union {
  volatile const uint8_t RBR;
  volatile uint8_t THR;
  volatile uint8_t DLL;
       uint32_t RESERVED0;
  };
  union {
  volatile uint8_t DLM;
  volatile uint32_t IER;
  };
  union {
  volatile const uint32_t IIR;
  volatile uint8_t FCR;
  };
  volatile uint8_t LCR;
       uint8_t RESERVED1[7];
  volatile const uint8_t LSR;
       uint8_t RESERVED2[7];
  volatile uint8_t SCR;
       uint8_t RESERVED3[3];
  volatile uint32_t ACR;
  volatile uint8_t ICR;
       uint8_t RESERVED4[3];
  volatile uint8_t FDR;
       uint8_t RESERVED5[7];
  volatile uint8_t TER;
       uint8_t RESERVED6[39];
  volatile uint32_t FIFOLVL;
} LPC_UART0_TypeDef;

typedef struct
{
  union {
  volatile const uint8_t RBR;
  volatile uint8_t THR;
  volatile uint8_t DLL;
       uint32_t RESERVED0;
  };
  union {
  volatile uint8_t DLM;
  volatile uint32_t IER;
  };
  union {
  volatile const uint32_t IIR;
  volatile uint8_t FCR;
  };
  volatile uint8_t LCR;
       uint8_t RESERVED1[3];
  volatile uint8_t MCR;
       uint8_t RESERVED2[3];
  volatile const uint8_t LSR;
       uint8_t RESERVED3[3];
  volatile const uint8_t MSR;
       uint8_t RESERVED4[3];
  volatile uint8_t SCR;
       uint8_t RESERVED5[3];
  volatile uint32_t ACR;
       uint32_t RESERVED6;
  volatile uint32_t FDR;
       uint32_t RESERVED7;
  volatile uint8_t TER;
       uint8_t RESERVED8[27];
  volatile uint8_t RS485CTRL;
       uint8_t RESERVED9[3];
  volatile uint8_t ADRMATCH;
       uint8_t RESERVED10[3];
  volatile uint8_t RS485DLY;
       uint8_t RESERVED11[3];
  volatile uint32_t FIFOLVL;
} LPC_UART1_TypeDef;


typedef struct
{
  volatile uint32_t SPCR;
  volatile const uint32_t SPSR;
  volatile uint32_t SPDR;
  volatile uint32_t SPCCR;
       uint32_t RESERVED0[3];
  volatile uint32_t SPINT;
} LPC_SPI_TypeDef;


typedef struct
{
  volatile uint32_t CR0;
  volatile uint32_t CR1;
  volatile uint32_t DR;
  volatile const uint32_t SR;
  volatile uint32_t CPSR;
  volatile uint32_t IMSC;
  volatile uint32_t RIS;
  volatile uint32_t MIS;
  volatile uint32_t ICR;
  volatile uint32_t DMACR;
} LPC_SSP_TypeDef;


typedef struct
{
  volatile uint32_t I2CONSET;
  volatile const uint32_t I2STAT;
  volatile uint32_t I2DAT;
  volatile uint32_t I2ADR0;
  volatile uint32_t I2SCLH;
  volatile uint32_t I2SCLL;
  volatile uint32_t I2CONCLR;
  volatile uint32_t MMCTRL;
  volatile uint32_t I2ADR1;
  volatile uint32_t I2ADR2;
  volatile uint32_t I2ADR3;
  volatile const uint32_t I2DATA_BUFFER;
  volatile uint32_t I2MASK0;
  volatile uint32_t I2MASK1;
  volatile uint32_t I2MASK2;
  volatile uint32_t I2MASK3;
} LPC_I2C_TypeDef;


typedef struct
{
  volatile uint32_t I2SDAO;
  volatile uint32_t I2SDAI;
  volatile uint32_t I2STXFIFO;
  volatile const uint32_t I2SRXFIFO;
  volatile const uint32_t I2SSTATE;
  volatile uint32_t I2SDMA1;
  volatile uint32_t I2SDMA2;
  volatile uint32_t I2SIRQ;
  volatile uint32_t I2STXRATE;
  volatile uint32_t I2SRXRATE;
  volatile uint32_t I2STXBITRATE;
  volatile uint32_t I2SRXBITRATE;
  volatile uint32_t I2STXMODE;
  volatile uint32_t I2SRXMODE;
} LPC_I2S_TypeDef;


typedef struct
{
  volatile uint32_t RICOMPVAL;
  volatile uint32_t RIMASK;
  volatile uint8_t RICTRL;
       uint8_t RESERVED0[3];
  volatile uint32_t RICOUNTER;
} LPC_RIT_TypeDef;


typedef struct
{
  volatile uint8_t ILR;
       uint8_t RESERVED0[7];
  volatile uint8_t CCR;
       uint8_t RESERVED1[3];
  volatile uint8_t CIIR;
       uint8_t RESERVED2[3];
  volatile uint8_t AMR;
       uint8_t RESERVED3[3];
  volatile const uint32_t CTIME0;
  volatile const uint32_t CTIME1;
  volatile const uint32_t CTIME2;
  volatile uint8_t SEC;
       uint8_t RESERVED4[3];
  volatile uint8_t MIN;
       uint8_t RESERVED5[3];
  volatile uint8_t HOUR;
       uint8_t RESERVED6[3];
  volatile uint8_t DOM;
       uint8_t RESERVED7[3];
  volatile uint8_t DOW;
       uint8_t RESERVED8[3];
  volatile uint16_t DOY;
       uint16_t RESERVED9;
  volatile uint8_t MONTH;
       uint8_t RESERVED10[3];
  volatile uint16_t YEAR;
       uint16_t RESERVED11;
  volatile uint32_t CALIBRATION;
  volatile uint32_t GPREG0;
  volatile uint32_t GPREG1;
  volatile uint32_t GPREG2;
  volatile uint32_t GPREG3;
  volatile uint32_t GPREG4;
  volatile uint8_t RTC_AUXEN;
       uint8_t RESERVED12[3];
  volatile uint8_t RTC_AUX;
       uint8_t RESERVED13[3];
  volatile uint8_t ALSEC;
       uint8_t RESERVED14[3];
  volatile uint8_t ALMIN;
       uint8_t RESERVED15[3];
  volatile uint8_t ALHOUR;
       uint8_t RESERVED16[3];
  volatile uint8_t ALDOM;
       uint8_t RESERVED17[3];
  volatile uint8_t ALDOW;
       uint8_t RESERVED18[3];
  volatile uint16_t ALDOY;
       uint16_t RESERVED19;
  volatile uint8_t ALMON;
       uint8_t RESERVED20[3];
  volatile uint16_t ALYEAR;
       uint16_t RESERVED21;
} LPC_RTC_TypeDef;


typedef struct
{
  volatile uint8_t WDMOD;
       uint8_t RESERVED0[3];
  volatile uint32_t WDTC;
  volatile uint8_t WDFEED;
       uint8_t RESERVED1[3];
  volatile const uint32_t WDTV;
  volatile uint32_t WDCLKSEL;
} LPC_WDT_TypeDef;


typedef struct
{
  volatile uint32_t ADCR;
  volatile uint32_t ADGDR;
       uint32_t RESERVED0;
  volatile uint32_t ADINTEN;
  volatile const uint32_t ADDR0;
  volatile const uint32_t ADDR1;
  volatile const uint32_t ADDR2;
  volatile const uint32_t ADDR3;
  volatile const uint32_t ADDR4;
  volatile const uint32_t ADDR5;
  volatile const uint32_t ADDR6;
  volatile const uint32_t ADDR7;
  volatile const uint32_t ADSTAT;
  volatile uint32_t ADTRM;
} LPC_ADC_TypeDef;


typedef struct
{
  volatile uint32_t DACR;
  volatile uint32_t DACCTRL;
  volatile uint16_t DACCNTVAL;
} LPC_DAC_TypeDef;


typedef struct
{
  volatile const uint32_t MCCON;
  volatile uint32_t MCCON_SET;
  volatile uint32_t MCCON_CLR;
  volatile const uint32_t MCCAPCON;
  volatile uint32_t MCCAPCON_SET;
  volatile uint32_t MCCAPCON_CLR;
  volatile uint32_t MCTIM0;
  volatile uint32_t MCTIM1;
  volatile uint32_t MCTIM2;
  volatile uint32_t MCPER0;
  volatile uint32_t MCPER1;
  volatile uint32_t MCPER2;
  volatile uint32_t MCPW0;
  volatile uint32_t MCPW1;
  volatile uint32_t MCPW2;
  volatile uint32_t MCDEADTIME;
  volatile uint32_t MCCCP;
  volatile uint32_t MCCR0;
  volatile uint32_t MCCR1;
  volatile uint32_t MCCR2;
  volatile const uint32_t MCINTEN;
  volatile uint32_t MCINTEN_SET;
  volatile uint32_t MCINTEN_CLR;
  volatile const uint32_t MCCNTCON;
  volatile uint32_t MCCNTCON_SET;
  volatile uint32_t MCCNTCON_CLR;
  volatile const uint32_t MCINTFLAG;
  volatile uint32_t MCINTFLAG_SET;
  volatile uint32_t MCINTFLAG_CLR;
  volatile uint32_t MCCAP_CLR;
} LPC_MCPWM_TypeDef;


typedef struct
{
  volatile uint32_t QEICON;
  volatile const uint32_t QEISTAT;
  volatile uint32_t QEICONF;
  volatile const uint32_t QEIPOS;
  volatile uint32_t QEIMAXPOS;
  volatile uint32_t CMPOS0;
  volatile uint32_t CMPOS1;
  volatile uint32_t CMPOS2;
  volatile const uint32_t INXCNT;
  volatile uint32_t INXCMP;
  volatile uint32_t QEILOAD;
  volatile const uint32_t QEITIME;
  volatile const uint32_t QEIVEL;
  volatile const uint32_t QEICAP;
  volatile uint32_t VELCOMP;
  volatile uint32_t FILTER;
       uint32_t RESERVED0[998];
  volatile uint32_t QEIIEC;
  volatile uint32_t QEIIES;
  volatile const uint32_t QEIINTSTAT;
  volatile const uint32_t QEIIE;
  volatile uint32_t QEICLR;
  volatile uint32_t QEISET;
} LPC_QEI_TypeDef;


typedef struct
{
  volatile uint32_t mask[512];
} LPC_CANAF_RAM_TypeDef;

typedef struct
{
  volatile uint32_t AFMR;
  volatile uint32_t SFF_sa;
  volatile uint32_t SFF_GRP_sa;
  volatile uint32_t EFF_sa;
  volatile uint32_t EFF_GRP_sa;
  volatile uint32_t ENDofTable;
  volatile const uint32_t LUTerrAd;
  volatile const uint32_t LUTerr;
  volatile uint32_t FCANIE;
  volatile uint32_t FCANIC0;
  volatile uint32_t FCANIC1;
} LPC_CANAF_TypeDef;

typedef struct
{
  volatile const uint32_t CANTxSR;
  volatile const uint32_t CANRxSR;
  volatile const uint32_t CANMSR;
} LPC_CANCR_TypeDef;

typedef struct
{
  volatile uint32_t MOD;
  volatile uint32_t CMR;
  volatile uint32_t GSR;
  volatile const uint32_t ICR;
  volatile uint32_t IER;
  volatile uint32_t BTR;
  volatile uint32_t EWL;
  volatile const uint32_t SR;
  volatile uint32_t RFS;
  volatile uint32_t RID;
  volatile uint32_t RDA;
  volatile uint32_t RDB;
  volatile uint32_t TFI1;
  volatile uint32_t TID1;
  volatile uint32_t TDA1;
  volatile uint32_t TDB1;
  volatile uint32_t TFI2;
  volatile uint32_t TID2;
  volatile uint32_t TDA2;
  volatile uint32_t TDB2;
  volatile uint32_t TFI3;
  volatile uint32_t TID3;
  volatile uint32_t TDA3;
  volatile uint32_t TDB3;
} LPC_CAN_TypeDef;


typedef struct
{
  volatile const uint32_t DMACIntStat;
  volatile const uint32_t DMACIntTCStat;
  volatile uint32_t DMACIntTCClear;
  volatile const uint32_t DMACIntErrStat;
  volatile uint32_t DMACIntErrClr;
  volatile const uint32_t DMACRawIntTCStat;
  volatile const uint32_t DMACRawIntErrStat;
  volatile const uint32_t DMACEnbldChns;
  volatile uint32_t DMACSoftBReq;
  volatile uint32_t DMACSoftSReq;
  volatile uint32_t DMACSoftLBReq;
  volatile uint32_t DMACSoftLSReq;
  volatile uint32_t DMACConfig;
  volatile uint32_t DMACSync;
} LPC_GPDMA_TypeDef;

typedef struct
{
  volatile uint32_t DMACCSrcAddr;
  volatile uint32_t DMACCDestAddr;
  volatile uint32_t DMACCLLI;
  volatile uint32_t DMACCControl;
  volatile uint32_t DMACCConfig;
} LPC_GPDMACH_TypeDef;


typedef struct
{
  volatile const uint32_t HcRevision;
  volatile uint32_t HcControl;
  volatile uint32_t HcCommandStatus;
  volatile uint32_t HcInterruptStatus;
  volatile uint32_t HcInterruptEnable;
  volatile uint32_t HcInterruptDisable;
  volatile uint32_t HcHCCA;
  volatile const uint32_t HcPeriodCurrentED;
  volatile uint32_t HcControlHeadED;
  volatile uint32_t HcControlCurrentED;
  volatile uint32_t HcBulkHeadED;
  volatile uint32_t HcBulkCurrentED;
  volatile const uint32_t HcDoneHead;
  volatile uint32_t HcFmInterval;
  volatile const uint32_t HcFmRemaining;
  volatile const uint32_t HcFmNumber;
  volatile uint32_t HcPeriodicStart;
  volatile uint32_t HcLSTreshold;
  volatile uint32_t HcRhDescriptorA;
  volatile uint32_t HcRhDescriptorB;
  volatile uint32_t HcRhStatus;
  volatile uint32_t HcRhPortStatus1;
  volatile uint32_t HcRhPortStatus2;
       uint32_t RESERVED0[40];
  volatile const uint32_t Module_ID;

  volatile const uint32_t OTGIntSt;
  volatile uint32_t OTGIntEn;
  volatile uint32_t OTGIntSet;
  volatile uint32_t OTGIntClr;
  volatile uint32_t OTGStCtrl;
  volatile uint32_t OTGTmr;
       uint32_t RESERVED1[58];

  volatile const uint32_t USBDevIntSt;
  volatile uint32_t USBDevIntEn;
  volatile uint32_t USBDevIntClr;
  volatile uint32_t USBDevIntSet;

  volatile uint32_t USBCmdCode;
  volatile const uint32_t USBCmdData;

  volatile const uint32_t USBRxData;
  volatile uint32_t USBTxData;
  volatile const uint32_t USBRxPLen;
  volatile uint32_t USBTxPLen;
  volatile uint32_t USBCtrl;
  volatile uint32_t USBDevIntPri;

  volatile const uint32_t USBEpIntSt;
  volatile uint32_t USBEpIntEn;
  volatile uint32_t USBEpIntClr;
  volatile uint32_t USBEpIntSet;
  volatile uint32_t USBEpIntPri;

  volatile uint32_t USBReEp;
  volatile uint32_t USBEpInd;
  volatile uint32_t USBMaxPSize;

  volatile const uint32_t USBDMARSt;
  volatile uint32_t USBDMARClr;
  volatile uint32_t USBDMARSet;
       uint32_t RESERVED2[9];
  volatile uint32_t USBUDCAH;
  volatile const uint32_t USBEpDMASt;
  volatile uint32_t USBEpDMAEn;
  volatile uint32_t USBEpDMADis;
  volatile const uint32_t USBDMAIntSt;
  volatile uint32_t USBDMAIntEn;
       uint32_t RESERVED3[2];
  volatile const uint32_t USBEoTIntSt;
  volatile uint32_t USBEoTIntClr;
  volatile uint32_t USBEoTIntSet;
  volatile const uint32_t USBNDDRIntSt;
  volatile uint32_t USBNDDRIntClr;
  volatile uint32_t USBNDDRIntSet;
  volatile const uint32_t USBSysErrIntSt;
  volatile uint32_t USBSysErrIntClr;
  volatile uint32_t USBSysErrIntSet;
       uint32_t RESERVED4[15];

  union {
  volatile const uint32_t I2C_RX;
  volatile uint32_t I2C_TX;
  };
  volatile const uint32_t I2C_STS;
  volatile uint32_t I2C_CTL;
  volatile uint32_t I2C_CLKHI;
  volatile uint32_t I2C_CLKLO;
       uint32_t RESERVED5[824];

  union {
  volatile uint32_t USBClkCtrl;
  volatile uint32_t OTGClkCtrl;
  };
  union {
  volatile const uint32_t USBClkSt;
  volatile const uint32_t OTGClkSt;
  };
} LPC_USB_TypeDef;


typedef struct
{
  volatile uint32_t MAC1;
  volatile uint32_t MAC2;
  volatile uint32_t IPGT;
  volatile uint32_t IPGR;
  volatile uint32_t CLRT;
  volatile uint32_t MAXF;
  volatile uint32_t SUPP;
  volatile uint32_t TEST;
  volatile uint32_t MCFG;
  volatile uint32_t MCMD;
  volatile uint32_t MADR;
  volatile uint32_t MWTD;
  volatile const uint32_t MRDD;
  volatile const uint32_t MIND;
       uint32_t RESERVED0[2];
  volatile uint32_t SA0;
  volatile uint32_t SA1;
  volatile uint32_t SA2;
       uint32_t RESERVED1[45];
  volatile uint32_t Command;
  volatile const uint32_t Status;
  volatile uint32_t RxDescriptor;
  volatile uint32_t RxStatus;
  volatile uint32_t RxDescriptorNumber;
  volatile const uint32_t RxProduceIndex;
  volatile uint32_t RxConsumeIndex;
  volatile uint32_t TxDescriptor;
  volatile uint32_t TxStatus;
  volatile uint32_t TxDescriptorNumber;
  volatile uint32_t TxProduceIndex;
  volatile const uint32_t TxConsumeIndex;
       uint32_t RESERVED2[10];
  volatile const uint32_t TSV0;
  volatile const uint32_t TSV1;
  volatile const uint32_t RSV;
       uint32_t RESERVED3[3];
  volatile uint32_t FlowControlCounter;
  volatile const uint32_t FlowControlStatus;
       uint32_t RESERVED4[34];
  volatile uint32_t RxFilterCtrl;
  volatile uint32_t RxFilterWoLStatus;
  volatile uint32_t RxFilterWoLClear;
       uint32_t RESERVED5;
  volatile uint32_t HashFilterL;
  volatile uint32_t HashFilterH;
       uint32_t RESERVED6[882];
  volatile const uint32_t IntStatus;
  volatile uint32_t IntEnable;
  volatile uint32_t IntClear;
  volatile uint32_t IntSet;
       uint32_t RESERVED7;
  volatile uint32_t PowerDown;
       uint32_t RESERVED8;
  volatile uint32_t Module_ID;
} LPC_EMAC_TypeDef;
# 30 "../../../os/hal/platforms/LPC17xx/hal_lld.h" 2
# 395 "../../../os/hal/platforms/LPC17xx/hal_lld.h"
  void hal_lld_init(void);
  void LPC17xx_clock_init(void);
# 44 "../../../os/hal/include/hal.h" 2


# 1 "../../../os/hal/include/io_channel.h" 1
# 78 "../../../os/hal/include/io_channel.h"
struct BaseChannelVMT {
  size_t (*write)(void *instance, const uint8_t *bp, size_t n); size_t (*read)(void *instance, uint8_t *bp, size_t n); msg_t (*put)(void *instance, uint8_t b); msg_t (*get)(void *instance); msg_t (*putt)(void *instance, uint8_t b, systime_t time); msg_t (*gett)(void *instance, systime_t time); size_t (*writet)(void *instance, const uint8_t *bp, size_t n, systime_t time); size_t (*readt)(void *instance, uint8_t *bp, size_t n, systime_t time);
};
# 89 "../../../os/hal/include/io_channel.h"
typedef struct {

  const struct BaseChannelVMT *vmt;
 
} BaseChannel;
# 247 "../../../os/hal/include/io_channel.h"
struct BaseAsynchronousChannelVMT {
  size_t (*write)(void *instance, const uint8_t *bp, size_t n); size_t (*read)(void *instance, uint8_t *bp, size_t n); msg_t (*put)(void *instance, uint8_t b); msg_t (*get)(void *instance); msg_t (*putt)(void *instance, uint8_t b, systime_t time); msg_t (*gett)(void *instance, systime_t time); size_t (*writet)(void *instance, const uint8_t *bp, size_t n, systime_t time); size_t (*readt)(void *instance, uint8_t *bp, size_t n, systime_t time);
};
# 258 "../../../os/hal/include/io_channel.h"
typedef struct {

  const struct BaseAsynchronousChannelVMT *vmt;
  EventSource event;
} BaseAsynchronousChannel;
# 47 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/io_block.h" 1
# 51 "../../../os/hal/include/io_block.h"
typedef enum {
  BLK_UNINIT = 0,
  BLK_STOP = 1,
  BLK_ACTIVE = 2,
  BLK_CONNECTING = 3,
  BLK_DISCONNECTING = 4,
  BLK_READY = 5,
  BLK_READING = 6,
  BLK_WRITING = 7,
  BLK_SYNCING = 8
} blkstate_t;




typedef struct {
  uint32_t blk_size;
  uint32_t blk_num;
} BlockDeviceInfo;
# 104 "../../../os/hal/include/io_block.h"
struct BaseBlockDeviceVMT {
  bool_t (*is_inserted)(void *instance); bool_t (*is_protected)(void *instance); bool_t (*connect)(void *instance); bool_t (*disconnect)(void *instance); bool_t (*read)(void *instance, uint32_t startblk, uint8_t *buffer, uint32_t n); bool_t (*write)(void *instance, uint32_t startblk, const uint8_t *buffer, uint32_t n); bool_t (*sync)(void *instance); bool_t (*get_info)(void *instance, BlockDeviceInfo *bdip);
};





typedef struct {

  const struct BaseBlockDeviceVMT *vmt;
  blkstate_t state;
} BaseBlockDevice;
# 48 "../../../os/hal/include/hal.h" 2


# 1 "../../../os/hal/include/mmcsd.h" 1
# 51 "../../../os/hal/include/hal.h" 2


# 1 "../../../os/hal/include/tm.h" 1
# 54 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/pal.h" 1
# 55 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/adc.h" 1
# 56 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/can.h" 1
# 57 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/ext.h" 1
# 58 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/gpt.h" 1
# 59 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/i2c.h" 1
# 60 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/icu.h" 1
# 61 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/mac.h" 1
# 62 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/pwm.h" 1
# 63 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/rtc.h" 1
# 64 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/serial.h" 1
# 100 "../../../os/hal/include/serial.h"
typedef enum {
  SD_UNINIT = 0,
  SD_STOP = 1,
  SD_READY = 2
} sdstate_t;




typedef struct SerialDriver SerialDriver;

# 1 "../../../os/hal/platforms/LPC17xx/serial_lld.h" 1
# 133 "../../../os/hal/platforms/LPC17xx/serial_lld.h"
typedef struct {



  uint32_t sc_speed;



  uint32_t sc_lcr;



  uint32_t sc_fcr;
} SerialConfig;
# 176 "../../../os/hal/platforms/LPC17xx/serial_lld.h"
extern SerialDriver SD1;





  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);
# 112 "../../../os/hal/include/serial.h" 2
# 124 "../../../os/hal/include/serial.h"
struct SerialDriverVMT {
  size_t (*write)(void *instance, const uint8_t *bp, size_t n); size_t (*read)(void *instance, uint8_t *bp, size_t n); msg_t (*put)(void *instance, uint8_t b); msg_t (*get)(void *instance); msg_t (*putt)(void *instance, uint8_t b, systime_t time); msg_t (*gett)(void *instance, systime_t time); size_t (*writet)(void *instance, const uint8_t *bp, size_t n, systime_t time); size_t (*readt)(void *instance, uint8_t *bp, size_t n, systime_t time);
};
# 135 "../../../os/hal/include/serial.h"
struct SerialDriver {

  const struct SerialDriverVMT *vmt;
  EventSource event; sdstate_t state; InputQueue iqueue; OutputQueue oqueue; uint8_t ib[16]; uint8_t ob[16]; LPC_UART_TypeDef *uart;
};
# 309 "../../../os/hal/include/serial.h"
  void sdInit(void);
  void sdObjectInit(SerialDriver *sdp, qnotify_t inotify, qnotify_t onotify);
  void sdStart(SerialDriver *sdp, const SerialConfig *config);
  void sdStop(SerialDriver *sdp);
  void sdIncomingDataI(SerialDriver *sdp, uint8_t b);
  msg_t sdRequestDataI(SerialDriver *sdp);
# 65 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/sdc.h" 1
# 66 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/spi.h" 1
# 67 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/uart.h" 1
# 68 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/usb.h" 1
# 69 "../../../os/hal/include/hal.h" 2


# 1 "../../../os/hal/include/mmc_spi.h" 1
# 72 "../../../os/hal/include/hal.h" 2
# 1 "../../../os/hal/include/serial_usb.h" 1
# 73 "../../../os/hal/include/hal.h" 2
# 208 "../../../os/hal/include/hal.h"
  void halInit(void);
# 38 "../../../os/hal/src/hal.c" 2
# 68 "../../../os/hal/src/hal.c"
void halInit(void) {

  hal_lld_init();
# 103 "../../../os/hal/src/hal.c"
  sdInit();
# 127 "../../../os/hal/src/hal.c"
  boardInit();
}
