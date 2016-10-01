/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
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

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <clock_time.h>

#include "../common/i2c_commands.h"

#include "main.h"
#include "beacon.h"

#define LINE_BUFF_SIZE 	128
#define CMD_LINE_SIZE 	32

/* local header functions */
void beacon_pull_opponent(void);

/* global variables */
int8_t beacon_connected=0;
int16_t link_id = 0;
int16_t error_id = 0;

char line_buff[LINE_BUFF_SIZE];
int8_t cmd_buff[CMD_LINE_SIZE];
uint16_t cmd_size = 0;


/*******************************************************************
 * BEACON WT11 COMMANDS 
 ******************************************************************/

/* reset wt11 of robot (mainboard) */
void beacon_cmd_wt11_local_reset(void)
{
	/* change to cmd mode */
	wait_ms(1000);
	uart_send(BEACON_UART,'+');	
	uart_send(BEACON_UART,'+');	
	uart_send(BEACON_UART,'+');	
	wait_ms(1000);
	
	uart_send(BEACON_UART,'\n');		
	uart_send(BEACON_UART,'\r');		

	/* reset wt11 */
	uart_send(BEACON_UART,'\r');	
	uart_send(BEACON_UART,'\n');	
	uart_send(BEACON_UART,'R');	
	uart_send(BEACON_UART,'E');	
	uart_send(BEACON_UART,'S');	
	uart_send(BEACON_UART,'E');	
	uart_send(BEACON_UART,'T');	
	uart_send(BEACON_UART,'\n');	
}

/* call to beacon wt11 to open connection */
void beacon_cmd_wt11_call(void)
{
	const char send_buf[] = "CALL 00:07:80:85:04:70 1 RFCOMM\n";		
	int16_t i=0;

	/* send call cmd */
	for(i=0; i<32; i++){
		uart_send(BEACON_UART, send_buf[i]);
	}	
}

/* close connection with beacon wt11 */
void beacon_cmd_wt11_close(void)
{
	/* change to cmd mode */
	wait_ms(1200);
	uart_send(BEACON_UART,'+');	
	uart_send(BEACON_UART,'+');	
	uart_send(BEACON_UART,'+');	
	wait_ms(1200);
	
	uart_send(BEACON_UART,'\n');		
	uart_send(BEACON_UART,'\r');		

	/* close conection */
	uart_send(BEACON_UART,'C');	
	uart_send(BEACON_UART,'L');	
	uart_send(BEACON_UART,'O');	
	uart_send(BEACON_UART,'S');	
	uart_send(BEACON_UART,'E');	
	uart_send(BEACON_UART,' ');	
	uart_send(BEACON_UART,'0');		
	uart_send(BEACON_UART,'\n');		
}	


/************************************************************
 * SEND AND RECEVE MESSAGES 
 ***********************************************************/

/* send command to beacon */
void beacon_send_cmd(int8_t *buff, uint16_t size) 
{
	int16_t i;

	/* check length */
	if(size > CMD_LINE_SIZE){
		ERROR(E_USER_BEACON, "Command size is too large");	
		return;
	}
		
	/* fill buffer */
	for(i=0; i<size; i++){
		cmd_buff[i] = buff[i];	
	}

	/* command size != 0 indicate 
    * that there is a command to send */		
	cmd_size = size;
}


/* parse line with sscanf */
void parse_line(char * buff) 
{
	int16_t ret;
//#define PARSE_OPPONENT_ASCII_VERSION
#ifdef PARSE_OPPONENT_ASCII_VERSION
	uint8_t flags;
	int16_t arg0, arg1, arg2, arg3;
	int32_t arg4;
	int32_t checksum;
#endif


	DEBUG(E_USER_BEACON,"from beacon: %s",buff);
	
	/* BEACON ANSWERS */

	/* beacon wt11 open link connection pass */
 	ret = sscanf(buff, "CONNECT %d RFCOMM 1", (int*)&link_id);
	if(ret == 1){
		beacon_connected = 1;
		NOTICE(E_USER_STRAT, "beacon wt11 link open PASS (%d)", link_id);						
	}

	/* beacon wt11 open link connection fails */
 	ret = sscanf(buff, "NO CARRIER %d ERROR %d RFC_CONNECTION_FAILED", (int*)&link_id, (int*)&error_id);
	if(ret == 2){
		beacon_connected = 0;
		ERROR(E_USER_STRAT, "beacon wt11 link open FAIL(%d,%d)", error_id, link_id);						
	}
	
#ifdef PARSE_OPPONENT_ASCII_VERSION
	/* opponent */
 	ret = sscanf(buff, "opponent is %d %d %d %d %lx",
 							 &arg0, &arg1, &arg2, &arg3, &arg4);
	if(ret == 5){

		DEBUG(E_USER_BEACON,"opponent ans parsed");

		/* check checksum */
		checksum  = arg0;
		checksum += arg1;
		checksum += arg2;
		checksum += arg3;

		if(checksum == arg4) {
			IRQ_LOCK(flags);
			beaconboard.opponent_x = (int16_t)arg0;
			beaconboard.opponent_y = (int16_t)arg1;		
			beaconboard.opponent_a = (int16_t)arg2;
			beaconboard.opponent_d = (int16_t)arg3;
			IRQ_UNLOCK(flags);		
		}		
		else
			NOTICE(E_USER_BEACON, "checksum error: %d %d %d %d %lx",
 					arg0, arg1, arg2, arg3, arg4);		
	}
#endif
	
}

/* process and parse line */
void line_char_in(char c)
{
	static uint8_t i = 0;

	if(c == '\r' || c == '\n'){
		if(i!=0){			
			line_buff[i] = '\0';
			parse_line(line_buff);
			i=0;
		}
	}
	else{
		line_buff[i++] = c;
		i &= 0x003F;
	}		
}

/* parse beacon opponent command raw answer */
uint8_t beacon_parse_opponent_answer(int16_t c)
{
	static uint8_t state = 0, i = 0;
	static int16_t opp_x=0, opp_y=0, opp_d=0, opp_a=0;
   #ifdef TWO_OPPONENTS
   static int16_t opp2_x=0, opp2_y=0, opp2_d=0, opp2_a=0;
   #endif
   
   #ifdef ROBOT_2ND
   static int16_t robot_2nd_x=0, robot_2nd_y=0, robot_2nd_d=0, robot_2nd_a=0;
   #endif

	static uint16_t checksum = 0;
	uint16_t local_checksum;
	uint8_t flags;

	switch(state) {
		case 0:
			/* parse header */
			if ((i == 0 && c == 't') ||
				 (i == 1 && c == 'n') ||
				 (i == 2 && c == 'e') ||
				 (i == 3 && c == 'n') ||
				 (i == 4 && c == 'o') ||
				 (i == 5 && c == 'p') ||
				 (i == 6 && c == 'p') ||
				 (i == 7 && c == 'o')) {
			
				i++;
			}
			else
				i = 0;
			
			if(i==8) {
				state = 1;
				DEBUG(E_USER_BEACON,"header detected");
			}
			break;

		case 1:
			/* read data */
			if(i==8)  { opp_x  = ((int16_t)c); 		}
			if(i==9)  { opp_x |= ((int16_t)c << 8);	}
			if(i==10) {	opp_y  = ((int16_t)c); 		}
			if(i==11) { opp_y |= ((int16_t)c << 8); }
			if(i==12) {	opp_a  = ((int16_t)c); 		}
			if(i==13) {	opp_a |= ((int16_t)c << 8); }
			if(i==14) {	opp_d  = ((int16_t)c); 		 } //printf("c(14) = %d\n\r", (int16_t)c);}
			if(i==15) {	opp_d |= ((int16_t)c << 8); } //printf("c(15) = %d\n\r", ((int16_t)c << 8));}

         /* Only one opponent */
#ifndef TWO_OPPONENTS
#ifndef ROBOT_2ND
         /* CHECKSUM */
			if(i==16) {	checksum  = ((uint16_t)c); 		}
			if(i==17) {	checksum |= ((uint16_t)c << 8); }
#endif
#endif

#ifdef TWO_OPPONENTS
			/* read data */
			if(i==16) { opp2_x  = ((int16_t)c); 		}
			if(i==17) { opp2_x |= ((int16_t)c << 8);	}
			if(i==18) {	opp2_y  = ((int16_t)c); 		}
			if(i==19) { opp2_y |= ((int16_t)c << 8); }
			if(i==20) {	opp2_a  = ((int16_t)c); 		}
			if(i==21) {	opp2_a |= ((int16_t)c << 8); }
			if(i==22) {	opp2_d  = ((int16_t)c); 		 } 
			if(i==23) {	opp2_d |= ((int16_t)c << 8); } 

         /*two opponents and 2nd robot*/
#ifdef ROBOT_2ND
			/* read data */
			if(i==24) { robot_2nd_x  = ((int16_t)c); 		}
			if(i==25) { robot_2nd_x |= ((int16_t)c << 8);	}
			if(i==26) {	robot_2nd_y  = ((int16_t)c); 		}
			if(i==27) { robot_2nd_y |= ((int16_t)c << 8); }
			if(i==28) {	robot_2nd_a  = ((int16_t)c); 		}
			if(i==29) {	robot_2nd_a |= ((int16_t)c << 8); }
			if(i==30) {	robot_2nd_d  = ((int16_t)c); 		 } 
			if(i==31) {	robot_2nd_d |= ((int16_t)c << 8); } 

         /* CHECKSUM */
			if(i==32) {	checksum  = ((uint16_t)c); 		}
			if(i==33) {	checksum |= ((uint16_t)c << 8); }
#endif

         /* Only two opponents */
#ifndef ROBOT_2ND
         /* CHECKSUM */
			if(i==24) {	checksum  = ((uint16_t)c); 		}
			if(i==25) {	checksum |= ((uint16_t)c << 8); }
#endif
#endif

         /*2nd robot and just one opponent*/
#ifndef TWO_OPPONENTS
#ifdef ROBOT_2ND
			/* read data */
			if(i==16) { robot_2nd_x  = ((int16_t)c); 		}
			if(i==17) { robot_2nd_x |= ((int16_t)c << 8);	}
			if(i==18) {	robot_2nd_y  = ((int16_t)c); 		}
			if(i==19) { robot_2nd_y |= ((int16_t)c << 8); }
			if(i==20) {	robot_2nd_a  = ((int16_t)c); 		}
			if(i==21) {	robot_2nd_a |= ((int16_t)c << 8); }
			if(i==22) {	robot_2nd_d  = ((int16_t)c); 		 } 
			if(i==23) {	robot_2nd_d |= ((int16_t)c << 8); } 

         /* CHECKSUM */
			if(i==24) {	checksum  = ((uint16_t)c); 		}
			if(i==25) {	checksum |= ((uint16_t)c << 8); }
#endif
#endif
			i++;


/* only one opponent*/
#ifndef TWO_OPPONENTS
#ifndef ROBOT_2ND
#define NUMBER 18
#endif
#endif

/*2nd robot and 2 opponents*/
#ifdef TWO_OPPONENTS
#ifdef ROBOT_2ND
#define NUMBER 34
#endif
/*2 opponents and no 2nd robot */
#ifndef ROBOT_2ND
#define NUMBER 26
#endif
#endif

/*2nd robot and 1 opponent*/
#ifndef TWO_OPPONENTS
#ifdef ROBOT_2ND
#define NUMBER 26
#endif
#endif
			if(i==NUMBER) {

				NOTICE(E_USER_BEACON,"data opp: %d %d %d %d\n\r", (int16_t)opp_x, (int16_t)opp_y, (int16_t)opp_a, (int16_t)opp_d);
#ifdef TWO_OPPONENTS
				NOTICE(E_USER_BEACON,"data opp2: %d %d %d %d\n\r", (int16_t)opp2_x, (int16_t)opp2_y, (int16_t)opp2_a, (int16_t)opp2_d);
#endif
#ifdef ROBOT_2ND
				NOTICE(E_USER_BEACON,"data robot 2nd: %d %d %d %d\n\r", (int16_t)robot_2nd_x, (int16_t)robot_2nd_y, (int16_t)robot_2nd_a, (int16_t)robot_2nd_d);
#endif
				NOTICE(E_USER_BEACON,"checksum: %d\n\r", (uint16_t)checksum);

				/* checksum */
				local_checksum  = (uint16_t)opp_x;
				local_checksum += (uint16_t)opp_y;
				local_checksum += (uint16_t)opp_a;
				local_checksum += (uint16_t)opp_d;
#ifdef TWO_OPPONENTS
				local_checksum += (uint16_t)opp2_x;
				local_checksum += (uint16_t)opp2_y;
				local_checksum += (uint16_t)opp2_a;
				local_checksum += (uint16_t)opp2_d;
#endif
#ifdef ROBOT_2ND
				local_checksum += (uint16_t)robot_2nd_x;
				local_checksum += (uint16_t)robot_2nd_y;
				local_checksum += (uint16_t)robot_2nd_a;
				local_checksum += (uint16_t)robot_2nd_d;
#endif

				/* save data */
				if(checksum == local_checksum) {

					IRQ_LOCK(flags);
					beaconboard.opponent_x = (int16_t)opp_x;
					beaconboard.opponent_y = (int16_t)opp_y;		
					beaconboard.opponent_a = (int16_t)opp_a;
					beaconboard.opponent_d = (int16_t)opp_d;
					IRQ_UNLOCK(flags);	

#ifdef TWO_OPPONENTS
					IRQ_LOCK(flags);
					beaconboard.opponent2_x = (int16_t)opp2_x;
					beaconboard.opponent2_y = (int16_t)opp2_y;		
					beaconboard.opponent2_a = (int16_t)opp2_a;
					beaconboard.opponent2_d = (int16_t)opp2_d;
					IRQ_UNLOCK(flags);	
#endif						
#ifdef ROBOT_2ND
					IRQ_LOCK(flags);
					beaconboard.robot_2nd_x = (int16_t)robot_2nd_x;
					beaconboard.robot_2nd_y = (int16_t)robot_2nd_y;		
					beaconboard.robot_2nd_a = (int16_t)robot_2nd_a;
					beaconboard.robot_2nd_d = (int16_t)robot_2nd_d;
					IRQ_UNLOCK(flags);	
#endif	
				}		
				else {
					ERROR(E_USER_STRAT, "opponent checksum fails");			
				}

				i=0;
				state = 0;
				return 1;
			}
			break;	

		default:
			i=0;
			state = 0;
			break;
	}

	return 0;
}


/*XXX NOT UPDATED for 2nd opponent and 2nd robot*/
/* for test pulling opponent possition */
void beacon_opponent_pulling(void)
{
#ifdef OPPONENT_PULLING_TEST
	static uint8_t state = 0, i = 0;
	static int16_t opp_x=0, opp_y=0, opp_d=0, opp_a=0, checksum=0;
	static microseconds us = 0;
	static int16_t c;
	uint8_t flags;

	/* return if event off */
	if((mainboard.flags & DO_OPP) == 0) {
		state = 0;
		i = 0;
		return;
	}

	switch(state) {
		case 0:
			if(time_get_us2() - us > (2*EVENT_PERIOD_BEACON_PULL)) {
				us = time_get_us2();
				DEBUG(E_USER_STRAT,"pull beacon");
				beacon_pull_opponent();
				i = 0;
				state = 1;
			}
			break;

		case 1:
			/* get data from UART */
			c = uart_recv_nowait(BEACON_UART);
			if(c == -1) {
				/* timeout */
				if(time_get_us2() - us > EVENT_PERIOD_BEACON_PULL) {
					us = time_get_us2();
					state = 0;
					i = 0;
				}
				break;
			}

			/* parse header */
			if ((i == 0 && c == 't') ||
				 (i == 1 && c == 'n') ||
				 (i == 2 && c == 'e') ||
				 (i == 3 && c == 'n') ||
				 (i == 4 && c == 'o') ||
				 (i == 5 && c == 'p') ||
				 (i == 6 && c == 'p') ||
				 (i == 7 && c == 'o')) {
			
				i++;
			}
			else {
				i = 0;
				break;
			}
			
			if(i==8) {
				state = 2;
				DEBUG(E_USER_STRAT,"header detected");
			}
			break;

		case 2:
			/* get data from UART */
			c = uart_recv_nowait(BEACON_UART);
			if(c == -1)
				break;

			/* read data */
			IRQ_LOCK(flags);
			if(i==8)  { opp_x  = ((uint16_t)c); 		}
			if(i==9)  { opp_x |= ((uint16_t)c << 8);	}
			if(i==10) {	opp_y  = ((uint16_t)c); 		}
			if(i==11) { opp_y |= ((uint16_t)c << 8); }
			if(i==12) {	opp_a  = ((uint16_t)c); 		}
			if(i==13) {	opp_a |= ((uint16_t)c << 8); }
			if(i==14) {	opp_d  = ((uint16_t)c); 		}
			if(i==15) {	opp_d |= ((uint16_t)c << 8); }

			if(i==16) {	checksum  = ((uint16_t)c); 		}
			if(i==17) {	checksum |= ((uint16_t)c << 8); }
			IRQ_UNLOCK(flags);

			i++;

			if(i==18) {

				NOTICE(E_USER_STRAT, "data: %d %d %d %d %d", (int16_t)opp_x, (int16_t)opp_y, (int16_t)opp_a, (int16_t)opp_d, (int16_t)checksum);

				/* save data */
				if(checksum == (opp_x + opp_y + opp_a + opp_d)) {
					IRQ_LOCK(flags);
					beaconboard.opponent_x = (int16_t)opp_x;
					beaconboard.opponent_y = (int16_t)opp_y;		
					beaconboard.opponent_a = (int16_t)opp_a;
					beaconboard.opponent_d = (int16_t)opp_d;
					IRQ_UNLOCK(flags);		
				}		
				else {
					NOTICE(E_USER_STRAT, "opponent checksum fails");

					/* flush enque */
					while(uart_recv_nowait(BEACON_UART) != -1);
		
				}

				i=0;
				state = 0;
			}
			break;	

		default:
			i=0;
			state = 0;
			break;
	}
#endif
}


/* send commands and parse answers */
void beacon_protocol(void * dummy)
{
	int16_t i;
	static uint8_t a = 0;
	volatile int16_t c = 0;
	uint8_t ret = 0;
	static microseconds pull_time_us = 0;
	
	/* beacon config commads */
	if((mainboard.flags & DO_OPP) == 0) {
		/* parse commands asnwers */
		while(c != -1) {	
			c = uart_recv_nowait(BEACON_UART);
			if(c != -1)
				line_char_in((char)(c & 0x00FF));	
		}

		/* send commands */
		if(cmd_size){	
			for(i=0; i<cmd_size; i++){
				uart_send(BEACON_UART, cmd_buff[i]);	
			}	
			cmd_size = 0;

			uart_send(BEACON_UART, '\n');
			uart_send(BEACON_UART, '\r');		
		}
	}
	/* beacon data pulling */
	else {

		/* led */
		a++;
		if (a & 0x4)
			LED3_TOGGLE();

		/* parse answers */
		c = uart_recv_nowait(BEACON_UART);
		while(c != -1){	
			ret = beacon_parse_opponent_answer(c);
			c = uart_recv_nowait(BEACON_UART);
		}

		/* request opponent possition */
		if((time_get_us2() - pull_time_us > 50000UL)) {
			beacon_pull_opponent();	
			pull_time_us = time_get_us2();
		}
	}	
}

/* init stuff */
void beacon_init(void)
{
}


/************************************************************
 * BEACON COMMANDS 
 ***********************************************************/

/* set color */
void beacon_cmd_color(void)
{
	int8_t buff[20];
	uint16_t size;
	
	if(mainboard.our_color == I2C_COLOR_YELLOW)
		size = sprintf((char *)buff,"\n\rcolor yellow");
	else
		size = sprintf((char *)buff,"\n\rcolor green");
	
	beacon_send_cmd(buff, size);
}


/* get opponent */
//void beacon_cmd_opponent(void)
//{
//	int8_t buff[32];
//	uint16_t size;
//	int16_t robot_x, robot_y, robot_a;
//	uint8_t flags;
//	
//	IRQ_LOCK(flags);
//	robot_x = position_get_x_s16(&mainboard.pos);
//	robot_y = position_get_y_s16(&mainboard.pos);
//	robot_a = position_get_a_deg_s16(&mainboard.pos);
//	IRQ_UNLOCK(flags);
//
//	size = sprintf((char *)buff,"opponent %d %d %d",
//								robot_x, robot_y, robot_a);
//
//	beacon_send_cmd(buff, size);
//}


/* beacon on */
void beacon_cmd_beacon_on(void)
{
	int8_t buff[] = "\n\rbeacon on";
	uint16_t size = 11;
	
	beacon_send_cmd(buff, size);
}

/* beacon on with watchdog */
void beacon_cmd_beacon_on_watchdog(void)
{
	int8_t buff[] = "\n\rbeacon watchdog_on";
	uint16_t size = 20;
	
	beacon_send_cmd(buff, size);
}

/* beacon off*/
void beacon_cmd_beacon_off(void)
{
	uint8_t flags;

	IRQ_LOCK(flags);
	mainboard.flags &= ~(DO_OPP);
	IRQ_UNLOCK(flags);

	int8_t buff[] = "\n\rbeacon off";
	uint16_t size = 12;
	
	beacon_send_cmd(buff, size);
}


/* pull opponent position */
void beacon_pull_opponent(void)
{
	char buff[32];
	uint16_t size;
	int16_t robot_x, robot_y, robot_a;
	uint8_t flags;
	uint8_t i;
	
	IRQ_LOCK(flags);
	robot_x = position_get_x_s16(&mainboard.pos);
	robot_y = position_get_y_s16(&mainboard.pos);
	robot_a = position_get_a_deg_s16(&mainboard.pos);
	IRQ_UNLOCK(flags);


	size = sprintf(buff,"opponent %d %d %d",
								robot_x, robot_y, robot_a);

	for(i=0; i<size; i++){
		uart_send(BEACON_UART, buff[i]);	
	}	

	uart_send(BEACON_UART, '\n');
	uart_send(BEACON_UART, '\r');		

}

