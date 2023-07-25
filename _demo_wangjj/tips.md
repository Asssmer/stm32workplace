    

# NVIC API
```C
void __disable_irq(void)

void __enable_irq(void)

void NVIC_SetPriorityGrouping(uint32_t priority_grouping)

void NVIC_EnableIRQ(IRQn_t IRQn)

void NVIC_DisableIRQ(IRQn_t IRQn)

uint32_t NVIC_GetPendingIRQ (IRQn_t IRQn)

void NVIC_SetPendingIRQ (IRQn_t IRQn)

void NVIC_ClearPendingIRQ (IRQn_t IRQn)

uint32_t NVIC_GetActive (IRQn_t IRQn)

void NVIC_SetPriority (IRQn_t IRQn, uint32_t priority)

uint2_t NVIC_GetPriority (IRQn_t IRQn)

void NVIC_SystemReset (void)
```
# BIT SET/RESET
```C
#define REG(x)  (*((volatile unsigned int *)(x)))

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
```