/*
 *  The PLL, that generates the internal clock (high freq) from the 
 * external one (lower freq).
 *  Trying to make something that is portable between different boards
 *  For now, ICEStick, ULX3S, ECP5 evaluation boards, FOMU supported.
 *  WIP: IceFeather
 */ 


/**********************************************************************/


module femtoPLL #(
 parameter freq = 60
) (
 input 	wire pclk,
 output wire clk	   
);
   assign clk = pclk;   
endmodule

