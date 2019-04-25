#ifndef _I2C_H
#define _I2C_H

#define _I2C_CR1	(*((volatile unsigned long *) 0x40005400)) /* Control register 1*/
#define _I2C_CR2	(*((volatile unsigned long *) 0x40005404)) /* Control register 2 */
#define _I2C_OAR1	(*((volatile unsigned long *) 0x40005408)) /* Own address register 1 */
#define _I2C_OAR2	(*((volatile unsigned long *) 0x4005540C)) /* Own address register 2 */
#define _I2C_DR		(*((volatile unsigned long *) 0x40055410)) /* Data register */
#define _I2C_SR1	(*((volatile unsigned long *) 0x40055414)) /* Status register 1 */
#define _I2C_SR2	(*((volatile unsigned long *) 0x40055418)) /* Status register 2 */
#define _I2C_CCR	(*((volatile unsigned long *) 0x4005541C)) /* Clock control register */
#define _I2C_TRISE	(*((volatile unsigned long *) 0x40055420)) /* TRISE register */

/* RCC_APB1ENR */

#endif	/* _I2C_H */
