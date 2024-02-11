/* INSERT NAME AND PENNKEY HERE */
/*
*   Name: Jin Qian & Yuxin Qian
*   PennKey: qian0928 & 
*/

`timescale 1ns / 1ns

// quotient = dividend / divisor

module divider_unsigned(input  wire [31:0] i_dividend,
                   input  wire [31:0] i_divisor,
                   output wire [31:0] o_remainder,
                   output wire [31:0] o_quotient);
      
    // TODO: your code here

    //creating an array of busses for the daisy chain interconnect between the div iters
    wire [31: 0] dividend[31: 0];
    wire [31: 0] remainder[31: 0];
    wire [31: 0] quotient[31: 0];

    //initialize the first case for input
    divu_1iter iter0(.i_dividend(i_dividend), 
                .i_divisor(i_divisor), 
                .i_remainder(32'b0), 
                .i_quotient(32'b0), 
                .o_dividend(dividend[0]), 
                .o_remainder(remainder[0]), 
                .o_quotient(quotient[0]));
    
    genvar i;
    for (i = 1; i < 32; i=i+1) begin
        divu_1iter iterdiv(.i_dividend(dividend[i - 1]), 
                    .i_divisor(i_divisor), 
                    .i_remainder(remainder[i -1]), 
                    .i_quotient(quotient[i - 1]), 
                    .o_dividend(dividend[i]), 
                    .o_remainder(remainder[i]), 
                    .o_quotient(quotient[i]));
    end

    wire divbyzero = i_divisor == 0;  
    assign o_remainder = (divbyzero) ? 32'b0 : remainder[31];
	assign o_quotient = (divbyzero) ? 32'b0 : quotient[31];

endmodule // lc4_divider

module divu_1iter (
    input  wire [31:0] i_dividend, 
    input  wire [31:0] i_divisor,
    input  wire [31:0] i_remainder,
    input  wire [31:0] i_quotient,
    output wire [31:0] o_dividend,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);
  /*
    for (int i = 0; i < 32; i++) {
        remainder = (remainder << 1) | ((dividend >> 31) & 0x1);
        if (remainder < divisor) {
            quotient = (quotient << 1);
        } else {
            quotient = (quotient << 1) | 0x1;
            remainder = remainder - divisor;
        }
        dividend = dividend << 1;
    }
    */

    // TODO: your code here
    
    wire compare;
    wire [31:0] new_remainder;

    assign new_remainder = (i_remainder << 1) | ((i_dividend >> 31) & 32'b1);
    assign compare = new_remainder < i_divisor; 

    assign o_quotient = compare ? (i_quotient << 1) : ((i_quotient << 1) | 32'b1);
    assign o_remainder = compare ? new_remainder : (new_remainder - i_divisor);
    assign  o_dividend =(i_dividend << 1);

endmodule
