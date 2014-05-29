/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2010)
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id$
 *
 *  Javier Baliñas Santos <javier@arc-robots.org>
 */


#include <i2c_slave_lite.h>

volatile uint8_t i2c_i;
uint8_t tx_data[I2C_BUFFER_SIZE];//={0x01,1};
uint8_t rx_data[I2C_BUFFER_SIZE];
volatile uint16_t rx_size;

void (*read_event)(uint8_t cmd, uint8_t *tx_data);
void (*write_event)(uint8_t cmd, uint8_t *rx_data, uint16_t size);

void i2c_register_write_event(void (*event)(uint8_t, uint8_t *, uint16_t))
{
	uint8_t flags;
	IRQ_LOCK(flags);
	write_event = event;
	IRQ_UNLOCK(flags);
}

void i2c_register_read_event(void (*event)(uint8_t, uint8_t *))
{
	uint8_t flags;
	IRQ_LOCK(flags);
	read_event = event;
	IRQ_UNLOCK(flags);
}


void i2c_init(uint8_t addr)
{
	uint8_t i;
	
	for(i=0; i<I2C_BUFFER_SIZE; i++){
		tx_data[i] = 0;
		rx_data[i] = 0;			
	}
	
	/* enable i2c module */
	I2C1CON = 0x8000;	
	
	/* set address of slave */
	I2C1ADD = addr;

	/* clear flag and enable interrupt */
	//_SI2C1IP = 6;
	_SI2C1IF = 0;
	_SI2C1IE = 1;	
}

void __attribute__((interrupt, no_auto_psv)) _SI2C1Interrupt(void)
{	
	uint8_t timeout;
	_SI2C1IF = 0;
					
	if (I2C1STATbits.D_A==0) {
		/* address byte */
		
		/* reset data pointer */
		i2c_i = 0;
		rx_size = 0;
		
		/* read address */
		timeout = 100;
		while(!I2C1STATbits.RBF && timeout !=0){
			timeout--;	
		}
    	
		I2C1STATbits.I2COV = 0;
		rx_data[i2c_i] = I2C1RCV;
		
		/* if is a read send the first data */
		if (I2C1STATbits.R_W) {
			I2C1TRN = tx_data[i2c_i++];   /* data transferred to I2C1TRN reg */    
	 	I2C1CONbits.SCLREL = 1;				/* Release the clock */
			
		}
	}
	/* data bytes */ 
	else {
		/* read from slave */
		if (I2C1STATbits.R_W) {
						
			I2C1TRN = tx_data[i2c_i++];   /* data transferred to I2C1TRN reg */     		I2C1CONbits.SCLREL = 1;			/* Release the clock */
     		rx_size = 0;
		}
		/* write to slave */ 
		else {
	
			timeout = 100;
			while(!I2C1STATbits.RBF && timeout !=0);  
	  		I2C1STATbits.I2COV = 0; 
    	
    		if(i2c_i < I2C_BUFFER_SIZE){   					rx_data[i2c_i++] = I2C1RCV;
			
				/* count the data rx and call write event*/
				rx_size++;
				write_event(rx_data[0], (uint8_t*)&rx_data[1], (rx_size-1));
			}
			else{
				/* XXX buffer overflow */
				rx_data[i2c_i] = I2C1RCV;
			}			
			
			/* prepare data for read from slave */
			if(i2c_i == 1){
				read_event(rx_data[0], tx_data);
			}		
		}

	}
	
	/* clear interrupt flag */
	_SI2C1IF = 0;

}

