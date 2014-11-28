/* 
 *   A device driver for the Texas Instruments
 *   Universal Paralllel Port (UPP)
 *  
 *   Author: Allan Jones <ajones@dlinnovations.com>
 *  
 *   Copyright (c) 2012, Digital Light Innovations.
 *   This file is licensed under the terms of the GNU General Public License
 *   version 2. This program is licensed "as is" without any warranty of any
 *   kind, whether express or implied.
 * 
 */
  
/*   NOTE WELL that the EVM schematic has Channel 0 is associated with Channel B,
 *   and Channel 1 is associated with Channel 1 (they are BACKWARDS!!!!):
 *    http://e2e.ti.com/support/dsp/omap_applications_processors/f/42/p/167887/615679.aspx#615679
 *  
 */

#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/fs.h>      
#include <linux/delay.h>      //for "mdelay(...)"
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/moduleparam.h>
#include <mach/da8xx.h>
#include <asm/sizes.h>
#include <asm/io.h>       
#include <mach/mux.h>

//SPRUH77A.PDF, Table 12-1. The Interrupt number assigned to the UPP module.
#define UPP_INTERRUPT   91

//SPRS586D.PDF, Table 2-4
#define DA850_UPP_BASE  0x01E16000

//SPRS586D.PDF, Table 5-117. Offsets from DA850_UPP_BASE
#define UPPCR           0x00000004
#define UPDLB           0x00000008
#define UPCTL           0x00000010
#define UPICR           0x00000014
#define UPTCR           0x0000001C
#define UPIER           0x00000024
#define UPIES           0x00000028
#define UPEOI           0x00000030
#define UPID0           0x00000040
#define UPID1           0x00000044
#define UPID2           0x00000048
#define UPIS0           0x00000050
#define UPIS1           0x00000054
#define UPIS2           0x00000058
#define UPQD0           0x00000060
#define UPQD1           0x00000064
#define UPQD2           0x00000068
#define UPQS0           0x00000070
#define UPQS1           0x00000074
#define UPQS2           0x00000078

//SPRS586D.PDF, Table 2-4
#define DA850_PSC1_BASE 0x01E27000 

//SPRUH77A.PDF, Table 9-7. 
//"Power and Sleep Controller 1" (PSC1)  Revision ID Register.
#define PSC_REVID       0x00000000

//SPRUH77A.PDF, Table 9-7. 
//"Power Domain Transition Status" Register.
#define PSC_PTSTAT      0x00000128

//SPRUH77A.PDF, Table 9-2, Table 9-7. 
//NOTE that in Table 9-2, UPP module has an LPSC number of 19...
#define PSC1_MDCTL_19   0x00000A4C  //0xA00 + (19*4). 

//SPRUH77A.PDF, Table 9-7. 
//"Power Domain Transition Command Register" Register.
#define PSC_PTCMD       0x00000120
 
//DMA Status Register bitmasks used in the ISR handler.... 
#define EOLI   16
#define EOWI   8
#define ERRI   4
#define UORI   2 
#define DPEI   1
#define EOLQ   (1 << 12)
#define EOWQ   (1 << 11)   
#define ERRQ   (1 << 10)
#define UORQ   (1 << 9)
#define DPEQ   (1 << 8)

//The DMA PARAMETERS 
#define UPP_BUF_SIZE       16384
#define UPP_RX_LINE_COUNT  4
#define UPP_RX_LINE_SIZE   4096
#define UPP_RX_LINE_OFFSET UPP_RX_LINE_SIZE
#define UPP_TX_LINE_COUNT  1
#define UPP_TX_LINE_SIZE   4096
#define UPP_TX_LINE_OFFSET 0

static void *txBuf;
static void *rxBuf; 
static void __iomem *pinmux_base = 0;
static void __iomem *upp_base    = 0;
static void __iomem *psc1_base   = 0;

//These may need to be changed to something more "serious", as I'm using them
//as a  type of semaphore/signal/etc between the ISR-handler 
//and the upp_read/upp_write functions...
static bool write_pending;
static bool read_pending;


//set to '1' when loading this module to use the Digital Loopback (DLB) 
//features,e.g:
//                "insmod UPP_driver.ko loopbackMode=1"
static int loopbackMode = 0;

module_param( loopbackMode, int, S_IRUGO);

// This function can be removed after active development is complete.
// simply "dumps" the contents of the relevant registers to the console
// for debugging/development purposes.
static void dump_Channel_regs( void )
{
   uint32_t regVal;

   regVal = ioread32( upp_base + UPIS0 );
   printk(KERN_INFO "UPIS0=0x%08X (Current address of Channel I).\n", regVal);
   regVal = ioread32( upp_base + UPIS1 );
   printk(KERN_INFO "UPIS1=0x%08X (Line-Count=%d and Byte-Count=%d).\n", regVal, regVal >> 16, regVal & 0xFFFF);
   regVal = ioread32( upp_base + UPIS2 );
   printk(KERN_INFO "UPIS2=0x%08X (ACTIVE=%d, PENDING=%d, WATERMARK=%d)\n", regVal, regVal & 0x01, regVal & 0x02, regVal >> 4);


   regVal = ioread32( upp_base + UPQS0 );
   printk(KERN_INFO "UPQS0=0x%08X (Current address of Channel Q).\n", regVal);
   regVal = ioread32( upp_base + UPQS1 );
   printk(KERN_INFO "UPQS1=0x%08X (Line-Count=%d and Byte-Count=%d).\n", regVal, regVal >> 16, regVal & 0xFFFF);
   regVal = ioread32( upp_base + UPQS2 );
   printk(KERN_INFO "UPQS2=0x%08X (ACTIVE=%d, PENDING=%d, WATERMARK=%d)\n", regVal, regVal & 0x01, regVal & 0x02, regVal >> 4);


   regVal = ioread32( upp_base + UPID0 );
   printk(KERN_INFO "UPID0=0x%08X (BASE ADDRESS)\n", regVal);
   regVal = ioread32( upp_base + UPID1 );
   printk(KERN_INFO "UPID1=0x%08X (LINE COUNT=%d, BYTE COUNT = %d)\n", regVal, regVal >> 16, regVal & 0xFFFF);
   regVal = ioread32( upp_base + UPID2 );
   printk(KERN_INFO "UPID2=0x%08X (CHANNEL LINE OFFSET)\n", regVal);

   regVal = ioread32( upp_base + UPQD0 );
   printk(KERN_INFO "UPQD0=0x%08X (BASE ADDRESS)\n", regVal);
   regVal = ioread32( upp_base + UPQD1 );
   printk(KERN_INFO "UPQD1=0x%08X (LINE COUNT=%d, BYTE COUNT = %d)\n", regVal, regVal >> 16, regVal & 0xFFFF);
   regVal = ioread32( upp_base + UPQD2 );
   printk(KERN_INFO "UPQD2=0x%08X (CHANNEL LINE OFFSET)\n", regVal);

}


   //SPRUGJ5B.PDF, Section 2.6.4
static irqreturn_t upp_ISR_handler(int irq, void *dev_id)
{
   uint32_t regVal, status;

   if (pinmux_base == 0) 
   {
      return IRQ_HANDLED;
   }

   status = ioread32( upp_base + UPIER );
   while (status &  0x1F1F )  //0x1F1F is an interrupt bit-mask 
   {
      //
      //DMA Channel I (Channel A), Transmitting data
      //
      if (status & EOLI) 
      {
         //Clear the interrupt. WRITING ZERO to any other bit has NO effect,
         //per SPRUH77A.PDF, section 33.3.9.
         iowrite32(EOLI, upp_base + UPIER );

         write_pending = false;
         printk(KERN_INFO "DMA:EOLI\n");
         dump_Channel_regs();
      }
      if (status & EOWI) 
      {
         //Clear the interrupt. WRITING ZERO to any other bit has NO effect,
         //per SPRUH77A.PDF, section 33.3.9.
         iowrite32(EOWI, upp_base + UPIER );

         write_pending = false;
         printk(KERN_INFO "DMA:EOWI\n");
         dump_Channel_regs();
      }
      if (status & ERRI) 
      {
         //Clear the interrupt. WRITING ZERO to any other bit has NO effect,
         //per SPRUH77A.PDF, section 33.3.9.
         iowrite32(ERRI, upp_base + UPIER );

         write_pending = false;
         printk(KERN_INFO "DMA Channel I *ERROR*.\n");
         dump_Channel_regs();
      }
      if (status & UORI) 
      {
         //Clear the interrupt. WRITING ZERO to any other bit has NO effect,
         //per SPRUH77A.PDF, section 33.3.9.
         iowrite32(UORI, upp_base + UPIER );

         write_pending = false;
         printk(KERN_INFO "DMA Channel I Underrun/Overflow.\n");
         dump_Channel_regs();
      }
      if (status & DPEI) 
      {
         //Clear the interrupt. WRITING ZERO to any other bit has NO effect,
         //per SPRUH77A.PDF, section 33.3.9.
         iowrite32(DPEI, upp_base + UPIER );

         write_pending = false;
         printk(KERN_INFO "DMA Channel I Programming Error (DPE).\n");
         dump_Channel_regs();
      }


      //
      //
      //DMA Channel Q (Channel B), Receiving data
      //
      //
      if (status & EOLQ) 
      {
         //Clear the interrupt. WRITING ZERO to any other bit has NO effect,
         //per SPRUH77A.PDF, section 33.3.9.
         iowrite32(EOLQ, upp_base + UPIER );

         read_pending = true;
         printk(KERN_INFO "DMA:EOLQ.\n");
         dump_Channel_regs();
      }
      if (status & EOWQ) 
      {
         //Clear the interrupt. WRITING ZERO to any other bit has NO effect,
         //per SPRUH77A.PDF, section 33.3.9.
         iowrite32(EOWQ, upp_base + UPIER );

         read_pending = true;
         printk(KERN_INFO "DMA:EOWQ.\n");
         dump_Channel_regs();
      }
      if (status & ERRQ) 
      {
         //Clear the interrupt. WRITING ZERO to any other bit has NO effect,
         //per SPRUH77A.PDF, section 33.3.9.
         iowrite32(ERRQ, upp_base + UPIER );

         read_pending = true;
         printk(KERN_INFO "DMA:ERRQ.\n");
         dump_Channel_regs();
      }
      if (status & UORQ) 
      {
         //Clear the interrupt. WRITING ZERO to any other bit has NO effect,
         //per SPRUH77A.PDF, section 33.3.9.
         iowrite32(UORQ, upp_base + UPIER );

         read_pending = true;
         printk(KERN_INFO "DMA Channel Q Underrun/Overflow.\n");
         dump_Channel_regs();
      }
      if (status & DPEQ) 
      {
         //Clear the interrupt. WRITING ZERO to any other bit has NO effect,
         //per SPRUH77A.PDF, section 33.3.9.
         iowrite32(DPEQ, upp_base + UPIER );

         read_pending = true;
         printk(KERN_INFO "DMA Channel Q Programming Error (DPE).\n");
         dump_Channel_regs();
      }

      //read again, and process if necessary.
      status = ioread32( upp_base + UPIER );
   }

   //Clear UPEOI to allow future calls to this function.
   regVal = ioread32( upp_base + UPEOI);
   regVal &= 0xFFFFFF00;
   iowrite32(regVal, pinmux_base + UPEOI);

   return IRQ_HANDLED;
}

static void pin_mux_set( int index, unsigned int bits )
{
   static DEFINE_SPINLOCK(mux_spin_lock);
   unsigned long flags;
   unsigned int offset;

   if ((index < 0) || (index > 19))
   {
      printk(KERN_INFO "pin_mux_set:index is out of range.\n");
      return;
   }

   if (!pinmux_base) 
   {
                                 //SRPUH77A.PDF,Table 11-3
      if ((pinmux_base = ioremap(DA8XX_SYSCFG0_BASE, SZ_4K)) == 0) 
      {
         printk(KERN_INFO "pin_mux_set:Cannot fetch pinmux_base.\n");
         return;
      }
   }

   offset = 0x120 + (index * 4);
   spin_lock_irqsave(&mux_spin_lock, flags);
   iowrite32(bits, pinmux_base + offset);
   spin_unlock_irqrestore(&mux_spin_lock, flags);

   //NOTE: do NOT "iounmap" the pinmux_base pointer, as it is used
   //      in the ISR_handler.....
}

/* 
//Reference configuration....
PINMUX0 => [0x24080000]                                                         
PINMUX1 => [0x00000004]                                                         
PINMUX2 => [0x88888880]                                                         
PINMUX3 => [0x88888888]                                                         
PINMUX4 => [0x11222288]                                                         
PINMUX5 => [0x00111111]                                                         
PINMUX6 => [0x00000080]                                                         
PINMUX7 => [0x00000000]                                                         
PINMUX8 => [0x88000000]                                                         
PINMUX9 => [0x00000000]                                                         
PINMUX10 => [0x88222222]                                                        
PINMUX11 => [0x88000000]                                                        
PINMUX12 => [0x88888888]                                                        
PINMUX13 => [0x44440000]                                                        
PINMUX14 => [0x44444400]                                                        
PINMUX15 => [0x44444444]                                                        
PINMUX16 => [0x44444444]                                                        
PINMUX17 => [0x44444444]                                                        
PINMUX18 => [0x00444444]                                                        
PINMUX19 => [0x00000000]  
*/

//"pin_mux_print()" is functionally correct. 
//Commented out simply to silence compiler warnings....

/*
void pin_mux_print( void )
{
   static DEFINE_SPINLOCK(mux_spin_lock);
   unsigned long flags;
   unsigned int regValue;
   unsigned int offset;
   int muxRegister;

   if (!pinmux_base) 
   {
                                 //SRPUH77A.PDF,Table 11-3
      if ((pinmux_base = ioremap(DA8XX_SYSCFG0_BASE, SZ_4K)) == 0) 
      {
         printk(KERN_INFO "pin_mux_print:Cannot fetch pinmux_base.\n");
         return;
      }
   }

   offset = 0x120;   //Pinmux0 is offset 0x120 in the SYSCFG0 Module. 
   muxRegister = 0;  //Simply for "pretty printing" the output.
   while (offset < 0x170 )
   {
      spin_lock_irqsave(&mux_spin_lock, flags);
      regValue = ioread32(pinmux_base + offset);
      spin_unlock_irqrestore(&mux_spin_lock, flags);
      printk(KERN_INFO "PINMUX%d => [0x%08x]\n", muxRegister++, regValue);
      offset += 4;
   }
}
*/

static void upp_pin_mux_init(void)
{
   pin_mux_set( 13, 0x44440000 );
   pin_mux_set( 14, 0x44444400 );
   pin_mux_set( 15, 0x44444444 );
   pin_mux_set( 16, 0x44444444 );
   pin_mux_set( 17, 0x44444444 );
   pin_mux_set( 18, 0x00444444 );

   //pin_mux_print();
}


   //SPRUGJ5B.PDF, Section 2.6.1.2.
static void upp_power_and_clocks( void )
{
   /* 
    * Refer to: 
    *   * Power and Sleep Controller (PSC), Chapter 9 in the TRM(SPRUH77A.PDF)
    *   * Device Clocking, Chapter 7 (esp Section 7.3.5) in the TRM. 
    *  
    *  
    */
    int regVal;


    if (!psc1_base) 
    {
       if ((psc1_base = ioremap(DA850_PSC1_BASE, SZ_4K)) == 0)
       {
          printk(KERN_INFO "upp_power_and_clocks:Cannot fetch psc1_base.\n");
          return;
       }
    }

    regVal = ioread32(psc1_base + PSC_REVID);

    //PSC Revision ID should be "44825A00" per SPRUH77A.PDF, section 9.6.1
    if (regVal == 0x44825A00) 
    {
       printk( KERN_INFO "PSC_REVID = 0x%08X....OK\n", regVal); 
    }
    else
    {
       printk( KERN_INFO "********ERROR: PSC_REVID = 0x%08X********\n", regVal); 
    }

    // SPRUH77A.PDF, 9.3.2.1, Table 9-6, 9.6.10 
    // wait for GOSTAT[0] in PSTAT to clear to 0 ("No transition in progress")
    while ( ioread32(psc1_base + PSC_PTSTAT) & 0x00000001 )
        ;

    //
    //SPRUH77A.PDF, 9.3.2.2,  9.6.19.
    //Set NEXT bit in MDCTL19 to Enable(3h).
    regVal  = ioread32( psc1_base + PSC1_MDCTL_19 );
    regVal |= 0x00000003;
    iowrite32(regVal, psc1_base + PSC1_MDCTL_19);


    //
    //SPRUH77A.PDF, 9.3.2.3,  9.6.9. 
    //Set the GO[0] bit in PTCMD to 1 to initiate power-domain transition
    regVal  = ioread32(psc1_base + PSC_PTCMD);
    regVal |= 0x00000001;
    iowrite32(regVal, psc1_base + PSC_PTCMD);

    //
    // SPRUH77A.PDF, 9.3.2.4 
    // Wait for GOSTAT[0] in PTSTAT to clear to 0
    while ( ioread32(psc1_base + PSC_PTSTAT) & 0x00000001 )
        ;

    iounmap( psc1_base );
    psc1_base = 0;
    printk( KERN_INFO "Clock and Power Configuration....OK\n"); 
}


   //SPRUGJ5B.PDF, Section 2.6.1.3, 2.6.1.4
static void upp_swrst( void )
{
    int32_t reg_val;

    if (!upp_base)
    {
       if ((upp_base = ioremap(DA850_UPP_BASE, SZ_4K)) == 0) 
       {
          printk(KERN_INFO "upp_swrst:Cannot fetch upp_base.\n");
          return;
       }
    }
    //Fetch the UPP ID for the sake of sanity....Should be "44231100"
    reg_val = ioread32( upp_base + 0 );
    if (reg_val == 0x44231100 ) 
    {
       printk( KERN_INFO "UPP_UPPID = 0x%08X....OK\n", reg_val);
    }
    else
    {
       printk( KERN_INFO "********ERROR: UPP_UPPID = 0x%08X********\n", reg_val); 
    }



    // SPRUH77A.PDF, Section 33.2.7.1.1, Table 33-12.
    // clear EN bit of UPPCR to (temporarily) disable the UPP. 
    reg_val = ioread32( upp_base + UPPCR );
    reg_val &= 0xfffffff7;
    iowrite32( reg_val, upp_base + UPPCR );

    // SPRUH77A.PDF, Section 33.2.7.1.2, Table 33-12.
    //poll "DMA Burst" (DB) bit of UPPCR to ensure DMA controller is idle
    while ( ioread32( upp_base + UPPCR ) & 0x00000080 )
        ;


    // SPRUH77A.PDF, Section 33.2.7.1.3, Table 33-12.
    // assert SWRST bit (bit 4) of UPPCR
    reg_val  = ioread32( upp_base + UPPCR );
    reg_val |= 0x00000010;
    iowrite32( reg_val, upp_base + UPPCR );

    //
    // wait at least 200 clock cycles 
    // (SPRUGJ5B.PDF, 2.6.1.4)
    mdelay( 200 );  // abitrary choice of 200ms


    // SPRUH77A.PDF, Section 33.2.7.1.4  --AND--
    // SPRUGJ5B.PDF, 2.6.1.4
    // clear SWRST bit (bit 4) of UPPCR
    reg_val = ioread32( upp_base + UPPCR );
    reg_val &= 0xffffffef;
    iowrite32( reg_val, upp_base + UPPCR );

    printk(KERN_INFO "upp_swrst....OK\n");
}


   //SPRUGJ5B.PDF, Section 2.6.1.5
static void upp_config( void )
{
   int32_t regVal;

   //-------------------------------------------------------------------------
   // UPPCTL - UPP Interface Channel Settings....SPRUH77A.PDF, Section 33.3.4.
   //        
   //        - DATA and XDATA Pin assignments to Channels A & B:
   //          Refer to SPRUGJ5B.PDF, Table 3: 
   //              
   //        ____PHYSICAL_PINS___|____CHANNEL_ASSIGNMENT___
   //          * DATA[7:0]       |       A[7:0]
   //          * DATA9[15:8]     |       B[7:0]
   //-------------------------------------------------------------------------
   regVal = 0x3;   //OPERATING MODE: Duplex 1: Channel A->XMIT, Channel B->RCV
   regVal |= 1 << 2; //CHANNEL MODE: Both Channel A & B are active
   iowrite32( regVal, upp_base + UPCTL );


   //-------------------------------------------------------------------------
   // UPPICR - signal enable, signal inversion, clk div (tx only), etc.
   //          SPRUH77A.PDF, Section 33.3.5
   //-------------------------------------------------------------------------
   regVal  = 0;        //Channel A: START is active-high
   regVal  = 0;        //Channel A: ENABLE is active-high
   regVal  = 0;        //Channel A: WAIT is active-high
   regVal |= 1 << 5;   //Channel A honors WAIT in transmit mode.
   //regVal |= 1 << 8;   //Channel A: Transmit-clock divisor by 2.
                       //Channel A:(CLKINVA) Signal on rising edge of clock
   regVal |= 1 << 13;  //Channel A:(TRISA) pins are High-impedence while idle


                       //Channel B: START is active-high
                       //Channel B: ENABLE is active-high
                       //Channel B: WAIT is active-high
   regVal |= 1 << 19;  //Channel B honors START in receive mode.
   regVal |= 1 << 20;  //Channel B honors ENABLE in recieve mode.
                       //Channel B:(CLKINVB) Signal on rising edge of clock
   iowrite32( regVal, upp_base + UPICR );


   //-------------------------------------------------------------------------
   // UPTCR - i/o tx thresh (tx only), dma read burst size
   //-------------------------------------------------------------------------
   regVal  = 0x03;            //DMA Channel I READ-threshold. 256 bytes (max)
   regVal |= 0x03 << 8;       //DMA Channel Q READ-threshold. 256 bytes
   regVal |= 0x03 << 16;      //    Channel A XMIT-threshold. 256 bytes
   regVal |= 0x03 << 24;      //    Channel B XMIT-threshold. 256 bytes
   iowrite32( regVal, upp_base + UPTCR );


   //-------------------------------------------------------------------------
   // UPPDLB - digital loopback
   //-------------------------------------------------------------------------


   if (loopbackMode == 0) 
   {
      //Disable ALL loopback paths.
      regVal  = 0;
   } else
   {
      //Enable A->B Loopback
      regVal  = 0;
      regVal |= 1 << 12;
      printk(KERN_INFO "***********WARNING! Using LOOPBACK mode**************\n");
   }
   iowrite32( regVal, upp_base + UPDLB );

   printk(KERN_INFO "upp_config....OK\n");
}

   //SPRUGJ5B.PDF, Section 2.6.1.6
static void upp_interrupt_enable( void )
{
   int32_t regVal, status;

   // Register the ISR before enabling the interrupts....
   status = request_irq( UPP_INTERRUPT, upp_ISR_handler, 0, "upp_ISR", 0 );
   if( status < 0 ) 
   {
        printk( KERN_INFO "upp_interrupt_enable: error requesting UPP IRQ %d\n", status );
        return;
    } 


   //-------------------------------------------------------------------------
   // UPIES - Interrupt Enable. Interrupt events will generate a CPU interrupt
   //-------------------------------------------------------------------------
   regVal  = 0x1F;            //Enable ALL interrupts for DMA Channel I
   regVal |= 0x1F << 8;       //Enable ALL interrupts for DMA Channel Q 
   iowrite32( regVal, upp_base + UPIES );

   printk(KERN_INFO "upp_interrupt_enable....OK\n");
}

   //SPRUGJ5B.PDF, Section 2.6.1.7
static void upp_enable( void )
{
    int32_t reg_val;

    // set EN bit in UPPCR. 
    // The EN bit (effectively disabling the UPP peripheral)...
    // was cleared in "upp_swrst()" function
    reg_val = ioread32( upp_base + UPPCR );
    reg_val |=  1 << 3;  
    iowrite32( reg_val, upp_base + UPPCR );

    printk(KERN_INFO "upp_enable....OK\n");
}

   //SPRUGJ5B.PDF, Section 2.6.1.8
static void upp_mem_alloc( void )
{
   /* NOTE: An experiment to try is to place the UPP buffers in L2 or L3
            RAM, to decrease the transfer times if there is a problem...
   */

   txBuf = kmalloc( UPP_BUF_SIZE, GFP_KERNEL  );
   if (!txBuf) 
   {
      printk(KERN_INFO "upp_mem_alloc: txBuf FAILED!\n");
      return;
   }

   rxBuf = kmalloc( UPP_BUF_SIZE, GFP_KERNEL  );
   if (!rxBuf) 
   {
      printk(KERN_INFO "upp_mem_alloc: rxBuf FAILED!\n");
      return;
   }

   //------------------------------------------------------------------------
   // Channel A (Tx), (DMA Channel I)
   //------------------------------------------------------------------------
   iowrite32( txBuf, upp_base + UPID0);  

   //------------------------------------------------------------------------
   // Channel B (Rx), (DMA Channel Q)
   //------------------------------------------------------------------------
   iowrite32( rxBuf, upp_base + UPQD0);  


   printk(KERN_INFO "upp_mem_alloc....OK\n");
}

static void upp_program_DMA_recv_channel( void )
{
   // wait for dma active/pending bits to clear
   while ( ioread32( upp_base + UPQS2 ) & 0x00000002 )
       ;

   //------------------------------------------------------------------------
   // Channel B (Rx), (DMA Channel Q)
   //------------------------------------------------------------------------
   iowrite32( rxBuf, upp_base + UPQD0);  
   iowrite32( ( UPP_RX_LINE_COUNT << 16 | UPP_RX_LINE_SIZE ), upp_base + UPQD1);
   iowrite32(  UPP_RX_LINE_OFFSET, upp_base + UPQD2);
}

static void upp_program_DMA_xmit_channel( void )
{
   // wait for dma active/pending bits to clear
   while ( ioread32( upp_base + UPIS2 ) & 0x00000002 )
        ;

   //------------------------------------------------------------------------
   // Channel A (Tx), (DMA Channel I)
   //------------------------------------------------------------------------
   iowrite32( txBuf, upp_base + UPID0);  
   iowrite32( ( UPP_TX_LINE_COUNT << 16 | UPP_TX_LINE_SIZE ), upp_base + UPID1);
   iowrite32( UPP_TX_LINE_OFFSET, upp_base + UPID2);
}



#if 0
   //SPRUGJ5B.PDF, Section 2.6.1.9
static void upp_program_DMA_channels( void )
{
   //-------------------------------------------------------------------------
   //This is where I program DMA window address,
   //Byte-Count, Line-Count, Line-offset, etc.
   //Look at SPRUGJ5B.PDF, Section 2.4.1.

   //upp_program_DMA_xmit_channel();
   //upp_program_DMA_recv_channel();

   iowrite32( txBuf, upp_base + UPID0);  
   iowrite32( rxBuf, upp_base + UPQD0);  

   printk(KERN_INFO "upp_program_DMA_channels: completed!\n");
}
#endif

//------------------------------------------------------------------------
// User-mode functions read/write/open/close, etc.
//------------------------------------------------------------------------


int upp_open( struct inode *iPtr, struct file *fPtr )
{
   int minor,major;
	
	minor=iminor(iPtr);
	major=imajor(iPtr);
	printk( KERN_INFO "upp_open: MAJOR(%d), MINOR(%d)\n", major, minor);

   return 0;
}

ssize_t upp_read( struct file *fPtr, char __user *buffer, size_t size, loff_t *offset )
{
   int retVal;
   int bytesRead = 0;
   void *bPtr = (void *)buffer;

   printk(KERN_INFO "upp_read:START: size=%d, offset=%d\n", size, (int)offset);



   //Put things back to their default value...
   upp_program_DMA_recv_channel();

   while (size > UPP_BUF_SIZE) 
   {

      //block on the semaphore/lock/signal/etc. The ISR handler sets 
      //"read_pending" to TRUE upon completing the DMA transfer....
      while (read_pending == false )
         ; 

      //retVal is ZERO on sucsess...
      retVal = copy_to_user(bPtr, rxBuf, size); 
      if(retVal)
      {
           printk(KERN_WARNING "upp_read: copy_to_user FAILED.\n");
           return -1;
      }
      read_pending  = false;
      size         -= UPP_BUF_SIZE; 
      bPtr         += UPP_BUF_SIZE; 
      bytesRead    += UPP_BUF_SIZE;
   }

   //
   //Write the remainder...This involves re-programming the DMA descriptors.
   //
   //
   // wait for dma active/pending bits to clear
   while ( ioread32( upp_base + UPQS2 ) & 0x00000002 )
        ;

   //1 line, remaining-bytes per line...
   iowrite32( (0x1 << 16 | size ), upp_base + UPQD1);
   //Program the "line offset" to ZERO to read only the first line
   // (See section 33.3.15 of SPRUH77A.PDF)
   iowrite32( 0, upp_base + UPQD2);


   printk(KERN_WARNING "upp_read: Hold on...........\n");

   //block on the semaphore/lock/signal/etc. The ISR handler sets 
   //"read_pending" to TRUE upon completing the DMA transfer....
   while (read_pending == false )
      ; 

   retVal = copy_to_user(bPtr, rxBuf, size); 
   if(retVal)
   {
        printk(KERN_WARNING "upp_read: copy_to_user FAILED.\n");
        return -1;
   }
   bytesRead += size;

   printk(KERN_INFO "upp_read:rxBuf =(%s), bytesRead=(%d)\n", (char *)rxBuf, bytesRead);

   printk(KERN_INFO "upp_read:DONE!\n");

   read_pending = false;
   return bytesRead;
}


ssize_t upp_write( struct file *fPtr, const char __user *buffer, size_t size, loff_t *offset )
{
   int retVal;
   int bytesWritten = 0;
   void *bPtr = (void *)buffer;
  
   printk(KERN_INFO "upp_write:START\n");

   if (!bPtr) 
   {
      printk(KERN_INFO "upp_write:ERROR: bPtr is null. Bailing....\n");
      return -1;
   }

   //Put things back to their default value...
   upp_program_DMA_xmit_channel();

   while (size > UPP_BUF_SIZE) 
   {

      //block on the semaphore/lock/signal/etc. The ISR handler sets 
      //"write_pending" to FALSE upon completing the DMA transfer....
      while (write_pending == true )
         ; 


      //retVal is ZERO on success...
      retVal = copy_from_user(txBuf, bPtr, UPP_BUF_SIZE);
      if(retVal)
      {
           printk(KERN_WARNING "upp_write: copy_from_user FAILED.\n");
           return -1;
      }
      write_pending = true;
      size         -= UPP_BUF_SIZE; 
      bPtr         += UPP_BUF_SIZE; 
      bytesWritten += UPP_BUF_SIZE;
   }

   //
   //Write the remainder...This involves re-programming the DMA descriptors.
   //
   //
   // wait for dma active/pending bits to clear
   while ( ioread32( upp_base + UPIS2 ) & 0x00000002 )
        ;

   //1 line, remaining-bytes per line...Bitmask "size" so that it is an even number....
   iowrite32( (0x1 << 16 | ( size & (~0x1) )), upp_base + UPID1);
   //Program the "line offset" to ZERO to send only the first line
   // (See section 33.3.15 of SPRUH77A.PDF)
   iowrite32( 0, upp_base + UPID2);


   //retVal is ZERO on success...
   retVal = copy_from_user(txBuf, bPtr, size);
   if(retVal)
   {
        printk(KERN_WARNING "upp_write: copy_from_user FAILED.\n");
        return -1;
   }
   bytesWritten += size;

   printk(KERN_INFO "upp_write:txBuf =(%s), bytesWritten=(%d)\n", (char *)txBuf, bytesWritten);

   printk(KERN_INFO "upp_write....OK\n");

   
   return bytesWritten;
}

int upp_release( struct inode *iPtr, struct file *fPtr )
{

   return 0;
}


static struct cdev *UPP_cdev;
static dev_t UPP_MajorMinorNumbers;

struct file_operations upp_fops = { 
  .owner    = THIS_MODULE,
  //.llseek   = no_llseek,
  //.poll     = upp_poll,
  .read     = upp_read,
  .write    = upp_write,
  //.ioctl	   = upp_ioctl,
  .open     = upp_open,
  //.release  = upp_release,
};


/*
 *  Return ZERO on success.
 *  
 */
static int __init upp_init(void)
{
   int retVal;

   write_pending = false;
   read_pending  = false;

   //SPRUGJ5B.PDF, Section 2.6.1.1
   upp_pin_mux_init();

   //SPRUGJ5B.PDF, Section 2.6.1.2.
   upp_power_and_clocks();

   //SPRUGJ5B.PDF, Section 2.6.1.3, 2.6.1.4
   upp_swrst();

   //SPRUGJ5B.PDF, Section 2.6.1.5
   upp_config();

   //SPRUGJ5B.PDF, Section 2.6.1.6
   upp_interrupt_enable();

   //SPRUGJ5B.PDF, Section 2.6.1.7
   upp_enable();

   //SPRUGJ5B.PDF, Section 2.6.1.8
   upp_mem_alloc();

   //SPRUGJ5B.PDF, Section 2.6.1.9
   //upp_program_DMA_channels();

   //
   //
   //At this point, everything should be properly initialized;
   //Now, I need to allow
   //character-device access: "open()", "read()", "write()", etc.
   //
   //

   UPP_MajorMinorNumbers = MKDEV( 0, 0);
   if ( (retVal = alloc_chrdev_region( &UPP_MajorMinorNumbers, 0, 1, "UPP" )) < 0)
   {
      printk(KERN_INFO "ERROR: Major/Minor number allocation failed.\n");
      return retVal;
   }


   UPP_cdev        = cdev_alloc();
   UPP_cdev->ops   = &upp_fops; 
   UPP_cdev->owner = THIS_MODULE;
   
   if (cdev_add( UPP_cdev, UPP_MajorMinorNumbers, 1) != 0) 
   {
      printk(KERN_INFO "ERROR: UPP driver NOT loaded. CDEV registration failed.\n");
   }
   else
   {
      printk(KERN_INFO "\nUPP Major:%d, Minor:%d\n", MAJOR(UPP_MajorMinorNumbers), MINOR(UPP_MajorMinorNumbers));
   }

   printk("UPP driver (1.7.0) succesfully installed.\n\n");
   return 0;
}

/*
 * 
 *  
 *  
 */
static void __exit upp_exit(void)
{
   uint32_t regVal;

    // SPRUH77A.PDF, Section 33.2.7.1.1, Table 33-12.
    // clear EN bit of UPPCR to disable the UPP. 
    regVal = ioread32( upp_base + UPPCR );
    regVal &= 0xfffffff7;
    iowrite32( regVal, upp_base + UPPCR );


    free_irq( UPP_INTERRUPT, 0);

    if (txBuf) 
    {
       kfree( txBuf );
       txBuf = 0;
    }
    if (rxBuf) 
    {
       kfree( rxBuf );
       rxBuf = 0;
    }

    cdev_del( UPP_cdev );
    unregister_chrdev_region( UPP_MajorMinorNumbers, 1);

    printk(KERN_INFO "UPP driver unloaded.\n");
}


MODULE_AUTHOR("Allan Jones, Digital Light Innovations");
MODULE_DESCRIPTION("OMAP-L138 UPP bus driver");
MODULE_LICENSE("GPL");
module_init(upp_init)
module_exit(upp_exit)
