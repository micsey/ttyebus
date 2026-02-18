//===============================================================================================================
//
// ttyebus - real time linux kernel module for the ebusd using the PL011 UART on a Rasperry Pi
//
// Copyright (C) 2017 Galileo53 <galileo53@gmx.at>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// This is a LINUX kernel module exclusively for the ebusd using the PL011 UART running on Raspberry Pi 3 and 4.
// The latency between receiving and transmitting of a character is nearly zero. This is achieved by
// disabling the hardware FIFO at the UART completely and using a ring-buffer managed at the interrupt
// handler of the "Receiver Holding Register" interrupt.
//
// With RASPI 3 we are replacing the original interrupt of ttyAMA0. With RASPI 4 the interrupt is shared between
// all 5 UARTs and so the interrupt of ttyebus is added to the shared list. Note that interrupt numbers in Raspbian
// (Debian) are re-ordered by some Linux-internal logic, so we must always see what interrupt is assigned at a specific
// RASPI / Raspbian version. This can be done by executing "cat /proc/interrupts" while ttyAMA0 is still active.
//
//===============================================================================================================
//
// Revision history:
// 2017-12-12   V1.1    Initial release
// 2017-12-18   V1.2    Added module description
// 2018-02-13   V1.3    Added more debug messages for IRQ read operations. Changed read timeout to 1 minute
// 2018-02-14   V1.4    Added poll to file operations
// 2018-03-21   V1.5    Fixed read buffer overrun issue
// 2019-06-16   V1.6    Changed IRQ for V4.19.42
// 2020-01-08   V1.7    Added support for RASPI4
// 2020-07-25	V1.8	Corrected set_fs(KERNEL_DS) for kernel 5.4
// 2024-05-07	v1.9	Changed ttyebus_raspi_model function for kernel >5.10
// 2026-02-18	v2.0	include configure script to get model and irq automatically 
//                      and redesigned for Kernel 6.x / Bookworm
//
//===============================================================================================================

#include "config.h"
#include <linux/fs.h>		// file stuff
#include <linux/kernel.h>	// printk()
#include <linux/errno.h>	// error codes
#include <linux/module.h>	// THIS MODULE
#include <linux/delay.h>	// udelay
#include <linux/interrupt.h>	// request_irq
#include <linux/miscdevice.h>	// misc_register
#include <linux/io.h>		// ioremap
#include <linux/spinlock.h>
#include <linux/wait.h>		// poll
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/of.h>

// #define DEBUG 1                  // if uncommented, will write some debug messages to /var/log/kern.log
// #define IRQDEBUG 1               // if uncommented, writes messages from the interrupt handler too (there are a lot of messages!)

#define DEVICE_NAME "ttyebus"

// Fallback für Makros aus dem Build-Script
#ifndef MODEL_VAL
#define MODEL_VAL 0
#endif
#ifndef IRQ_VAL
#define IRQ_VAL 0
#endif

static unsigned int RaspiModel = MODEL_VAL;
static unsigned int UartIrq = IRQ_VAL;
static void __iomem *GpioAddr;
static void __iomem *UartAddr;
static int DeviceOpen = 0;
static wait_queue_head_t WaitQueue;
static spinlock_t SpinLock;

// Ring-Buffer
enum { RX_BUFF_SIZE = 128 };
static volatile unsigned int RxTail = 0;
static volatile unsigned int RxHead = 0;
static unsigned int RxBuff[RX_BUFF_SIZE];

enum { TX_BUFF_SIZE = 64 };
static volatile unsigned int TxTail = TX_BUFF_SIZE;
static volatile unsigned int TxHead = TX_BUFF_SIZE;
static unsigned char TxBuff[TX_BUFF_SIZE];

#ifdef IRQDEBUG
static int IrqCounter = 0;
#endif

// Adressen (bleiben identisch für RPi 3 und 4)
#define RASPI_1_PERI_BASE    0x20000000
#define RASPI_23_PERI_BASE   0x3F000000
#define RASPI_4_PERI_BASE    0xFE000000
#define GPIO_BASE            0x00200000
#define UART0_BASE           0x00201000

// UART Register Offsets
#define UART_DATA         0x00
#define UART_RX_ERR       0x04
#define UART_FLAG         0x18
#define UART_LINE_CTRL    0x2C
#define UART_CTRL         0x30
#define UART_INT_MASK     0x38
#define UART_INT_STAT     0x40
#define UART_INT_CLR      0x44
#define UART_INT_BAUD     0x24
#define UART_FRAC_BAUD    0x28

#define INT_RX            (1 << 4)
#define INT_TX            (1 << 5)
#define UARTCR_UART_ENABLE (1 << 0)
#define UARTCR_TX_ENABLE   (1 << 8)
#define UARTCR_RX_ENABLE   (1 << 9)
#define UARTCR_RTS         (1 << 11)
#define UART_LCR_8_BITS    (3 << 5)

// ===============================================================================================
//
//                                    ttyebus_set_gpio_mode
//
// ===============================================================================================
//
// Parameter:
//      Gpio                Number of the GPIO port
//      Function            one of GPIO_INPUT, GPIO_OUTPUT, GPIO_ALT_0, etc.
//
// Returns:
//
// Description:
//      Set the mode for the GPIO port. Especially in this program GPIO_ALT_0 at port 14, 15 will
//      connect the ports to the UART Rx and Tx
//
// ===============================================================================================
static void ttyebus_set_gpio_mode(unsigned int Gpio, unsigned int Function) {
    unsigned int RegOffset = (Gpio / 10) << 2;
    unsigned int Bit = (Gpio % 10) * 3;
    u32 val = ioread32(GpioAddr + RegOffset);
    val &= ~(0x7 << Bit);
    val |= ((Function & 0x7) << Bit);
    iowrite32(val, GpioAddr + RegOffset);
}

// ===============================================================================================
//
//                                    ttyebus_irq_handler
//
// ===============================================================================================
//
// Parameter:
//
// Returns:
//
// Description:
//      Fired on interrupt. If data is in the receiver holding register, transfer it to the ring
//      buffer. If transmitter holding register has become empty, fill it with another data from
//      the linear buffer.
//
// ===============================================================================================
static irqreturn_t ttyebus_irq_handler(int irq, void* dev_id) {
    u32 IntStatus = ioread32(UartAddr + UART_INT_STAT);
    
#ifdef IRQDEBUG
    printk(KERN_NOTICE "ttyebus: IRQ %d called. RxHead=%d, RxTail=%d, TxHead=%d, TxTail=%d", IrqCounter, RxHead, RxTail, TxHead, TxTail);
#endif

    if (IntStatus & INT_RX) {
        iowrite32(INT_RX, UartAddr + UART_INT_CLR);
        u32 data = ioread32(UartAddr + UART_DATA);
        
        spin_lock(&SpinLock);
        unsigned int next = (RxHead + 1) % RX_BUFF_SIZE;

        // see if the buffer will be full after this interrupt
        // ===================================================
        if (next != RxTail) {
            RxBuff[RxHead] = data;
            RxHead = next;
#ifdef IRQDEBUG
            printk(KERN_NOTICE "ttyebus: IRQ: One byte received. RxHead=%d, RxTail=%d", RxHead, RxTail);
#endif
        } else {
// buffer overrun. do nothing. just discard the data.
#ifdef IRQDEBUG
            printk(KERN_NOTICE "ttyebus: IRQ: Buffer overrun. RxHead=%d, RxTail=%d", RxHead, RxTail);
#endif
        }
        spin_unlock(&SpinLock);
        wake_up_interruptible(&WaitQueue);
    }

    if (IntStatus & INT_TX) {
        iowrite32(INT_TX, UartAddr + UART_INT_CLR);
        spin_lock(&SpinLock);
        if (TxTail < TxHead) {
#ifdef IRQDEBUG
            printk(KERN_NOTICE "ttyebus: IRQ: Transmitting one byte. TxHead=%d, TxTail=%d", TxHead, TxTail);
#endif
            iowrite32(TxBuff[TxTail++], UartAddr + UART_DATA);
        } else {
#ifdef IRQDEBUG
            printk(KERN_NOTICE "ttyebus: IRQ: Stopping Tx Interrupt. TxHead=%d, TxTail=%d", TxHead, TxTail);
#endif
            u32 mask = ioread32(UartAddr + UART_INT_MASK);
            iowrite32(mask & ~INT_TX, UartAddr + UART_INT_MASK);
        }
        spin_unlock(&SpinLock);
    }

#ifdef IRQDEBUG
    printk(KERN_NOTICE "ttyebus: IRQ %d exit. RxHead=%d, RxTail=%d, TxHead=%d, TxTail=%d", IrqCounter, RxHead, RxTail, TxHead, TxTail);
    IrqCounter++;
#endif

    return IRQ_HANDLED;
}

// ===============================================================================================
//
//                                    ttyebus_poll
//
// ===============================================================================================
//
// Parameter:
//      file_ptr            Pointer to the open file
//      wait                Timeout structure
//
// Returns:
//      POLLIN              Data is available
//
// Description:
//      Probe the receiver if some data available. Return after timeout anyway.
//
// ===============================================================================================
static __poll_t ttyebus_poll(struct file* file, poll_table* wait) {
#ifdef DEBUG
    printk(KERN_NOTICE "ttyebus: Poll request");
#endif
    poll_wait(file, &WaitQueue, wait);
    if (RxTail != RxHead)
        {
#ifdef DEBUG
        printk(KERN_NOTICE "ttyebus: Poll succeeded. RxHead=%d, RxTail=%d", RxHead, RxTail);
#endif
        return EPOLLIN | EPOLLRDNORM;
        }
    else
        {
#ifdef DEBUG
        printk(KERN_NOTICE "ttyebus: Poll timeout");
#endif
        return 0;
        }    
}

// ===============================================================================================
//
//                                    ttyebus_read
//
// ===============================================================================================
//
// Parameter:
//      file                Pointer to the open file
//      user_buffer         Buffer in user space where to receive the data
//      count               Number of bytes to read
//      offset              Pointer to a counter that can hold an offset when reading chunks
//
// Returns:
//      Number of bytes read
//
// Description:
//      Called when a process, which already opened the dev file, attempts to read from it, like
//      "cat /dev/ttyebus"
//
// ===============================================================================================
static ssize_t ttyebus_read(struct file* file, char __user* user_buffer, size_t count, loff_t* offset) {
    unsigned int NumBytes = 0;
    unsigned char buffer[RX_BUFF_SIZE];
    unsigned long flags;

#ifdef DEBUG
    printk(KERN_NOTICE "ttyebus: Read request with offset=%d and count=%u", (int)*offset, (unsigned int)count);
#endif

    if (wait_event_interruptible_timeout(WaitQueue, RxTail != RxHead, msecs_to_jiffies(60000)) <= 0) {
#ifdef DEBUG
        printk(KERN_NOTICE "ttyebus: Read timeout");
#endif
		return -ETIMEDOUT;
        }

#ifdef IRQDEBUG
    printk(KERN_NOTICE "ttyebus: Read event. RxHead=%d, RxTail=%d", RxHead, RxTail);
#endif

    spin_lock_irqsave(&SpinLock, flags);
    while (RxTail != RxHead && NumBytes < count && NumBytes < RX_BUFF_SIZE) {
        buffer[NumBytes++] = (unsigned char)RxBuff[RxTail];
        RxTail = (RxTail + 1) % RX_BUFF_SIZE;
    }
    spin_unlock_irqrestore(&SpinLock, flags);

    if (copy_to_user(user_buffer, buffer, NumBytes))
        return -EFAULT;

#ifdef DEBUG
    printk(KERN_NOTICE "ttyebus: Read exit with %d bytes read", NumBytes);
#endif

    return NumBytes;
}

// ===============================================================================================
//
//                                    ttyebus_write
//
// ===============================================================================================
//
// Parameter:
//      file                Pointer to the open file
//      user_buffer         Buffer in user space where to receive the data
//      count               Number of bytes to write
//      offset              Pointer to a counter that can hold an offset when writing chunks
//
// Returns:
//      Number of bytes written
//
// Description:
//      Called when a process, which already opened the dev file, attempts to write to it, like
//      "echo "hello" > /dev/ttyebus"
//
// ===============================================================================================
static ssize_t ttyebus_write(struct file* file, const char __user* user_buffer, size_t count, loff_t* offset) {
    unsigned long flags;
    size_t actual = (count > TX_BUFF_SIZE) ? TX_BUFF_SIZE : count;

#ifdef DEBUG
    printk(KERN_NOTICE "ttyebus: Write request with offset=%d and count=%u", (int)*offset, (unsigned int)count);
#endif
#ifdef IRQDEBUG
    printk(KERN_NOTICE "ttyebus: Write request. TxHead=%d, TxTail=%d", TxHead, TxTail);
#endif

    if (copy_from_user(TxBuff, user_buffer, actual))
        return -EFAULT;

    spin_lock_irqsave(&SpinLock, flags);
    TxTail = 1;
    TxHead = actual;
    iowrite32(TxBuff[0], UartAddr + UART_DATA);
    u32 mask = ioread32(UartAddr + UART_INT_MASK);
    iowrite32(mask | INT_TX, UartAddr + UART_INT_MASK);
    spin_unlock_irqrestore(&SpinLock, flags);

#ifdef DEBUG
    printk(KERN_NOTICE "ttyebus: Write exit with %u bytes written", (unsigned int)count);
#endif

    return actual;
}

// ===============================================================================================
//
//                                    ttyebus_open
//
// ===============================================================================================
//
// Parameter:
//
// Returns:
//
// Description:
//      Called when a process tries to open the device file, like "cat /dev/ttyebus"
//
// ===============================================================================================
static int ttyebus_open(struct inode* inode, struct file* file) {

    if (DeviceOpen) return -EBUSY;
    DeviceOpen++;

    iowrite32(0, UartAddr + UART_CTRL); // Disable UART
    RxTail = RxHead = 0;
    
    // GPIO Setup (TX=14, RX=15)
    ttyebus_set_gpio_mode(14, 4); // ALT0
    ttyebus_set_gpio_mode(15, 4); // ALT0

    iowrite32(0x7FF, UartAddr + UART_INT_CLR);
    iowrite32(3000000 / 2400, UartAddr + UART_INT_BAUD);
    iowrite32(0, UartAddr + UART_FRAC_BAUD);
    iowrite32(UART_LCR_8_BITS, UartAddr + UART_LINE_CTRL);
    iowrite32(INT_RX, UartAddr + UART_INT_MASK);
    
    iowrite32(UARTCR_UART_ENABLE | UARTCR_TX_ENABLE | UARTCR_RX_ENABLE | UARTCR_RTS, UartAddr + UART_CTRL);

    return 0;
}

// ===============================================================================================
//
//                                    ttyebus_release
//
// ===============================================================================================
//
// Parameter:
//
// Returns:
//
// Description:
//      Called when a process closes the device file.
//
// ===============================================================================================
static int ttyebus_release(struct inode* inode, struct file* file) {
    iowrite32(0, UartAddr + UART_CTRL);
    DeviceOpen = 0;
    return 0;
}

// file operations with this kernel module
static const struct file_operations ttyebus_fops = {
    .owner = THIS_MODULE,
    .open = ttyebus_open,
    .release = ttyebus_release,
    .read = ttyebus_read,
    .write = ttyebus_write,
    .poll = ttyebus_poll,
};

static struct miscdevice ttyebus_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_NAME,
    .fops = &ttyebus_fops,
    .mode = 0666,
};

// --- Init & Exit ---

static int __init ttyebus_init(void) {
    u32 peri_base;
    int ret;

    if (RaspiModel == 1) peri_base = RASPI_1_PERI_BASE;
    else if (RaspiModel == 4) peri_base = RASPI_4_PERI_BASE;
    else peri_base = RASPI_23_PERI_BASE;

    GpioAddr = ioremap(peri_base + GPIO_BASE, SZ_4K);
    UartAddr = ioremap(peri_base + UART0_BASE, SZ_4K);

    init_waitqueue_head(&WaitQueue);
    spin_lock_init(&SpinLock);

    ret = misc_register(&ttyebus_misc);
    if (ret) return ret;

    // IRQ Request
    ret = request_irq(UartIrq, ttyebus_irq_handler, (RaspiModel == 4) ? IRQF_SHARED : 0, "ttyebus", (RaspiModel == 4) ? &ttyebus_misc : NULL);
    if (ret) {
        misc_deregister(&ttyebus_misc);
        return ret;
    }

    return 0;
}

static void __exit ttyebus_exit(void) {
    free_irq(UartIrq, (RaspiModel == 4) ? &ttyebus_misc : NULL);
    misc_deregister(&ttyebus_misc);
    if (GpioAddr) iounmap(GpioAddr);
    if (UartAddr) iounmap(UartAddr);
}

module_init(ttyebus_init);
module_exit(ttyebus_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Galileo53 / Fix for Bookworm / micsey");
MODULE_DESCRIPTION("Ebusd UART driver for PL011");
MODULE_VERSION("2.0");
