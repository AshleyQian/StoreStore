/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 * This code is for UART testing
 */

#include <asf.h>
#include <string.h>
#include "conf_clocks.h"
#include "delay.h"

#define UART_MODULE       SERCOM4
#define UART_SERCOM_MUX   USART_RX_3_TX_2_XCK_3
#define UART_SERCOM_GCLK  SERCOM4_GCLK_ID_CORE
#define BUFFER_SIZE 64

#define UART_TX_PIN PINMUX_PB10D_SERCOM4_PAD2 //SAM W25 TX line
#define UART_RX_PIN PINMUX_PB11D_SERCOM4_PAD3 //SAM W25 RX line

static uint8_t rx_buffer[BUFFER_SIZE];
static uint8_t tx_buffer[BUFFER_SIZE] = "UART!\r\n";

static struct usart_module usart_instance;
static int receive_flag = 0;

// UART read callback function (receive data)
static void usart_read_callback(struct usart_module *const module) {
	usart_read_buffer_job(&usart_instance, rx_buffer, BUFFER_SIZE);
	// Set receive flag while reading new data comes in
	if (port_pin_get_input_level(BUTTON_0_PIN) == !BUTTON_0_ACTIVE) receive_flag = 1;
}

// Configure UART communication
static void configure_uart(void) {
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);

	// Configure USART with the settings defined above
	config_usart.mux_setting = UART_SERCOM_MUX;
	config_usart.pinmux_pad0 = UART_TX_PIN; // TX pin
	config_usart.pinmux_pad1 = UART_RX_PIN; // RX pin

	// The program will run while(1) empty loop when initialization was unsuccessful
	while (usart_init(&usart_instance, UART_MODULE, &config_usart) != STATUS_OK) {} 
	usart_enable(&usart_instance);

	// Register and enable USART callback
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
	usart_register_callback(&usart_instance, usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	
	// Start reception
	usart_read_buffer_job(&usart_instance, rx_buffer, BUFFER_SIZE);
}


int main (void)
{
	// Initialize system and modules
	system_init();
	delay_init();
	configure_uart();

	usart_write_buffer_job(&usart_instance, tx_buffer, strlen((const char *)tx_buffer));

	while (1) {
		/* Is button pressed? */
		if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
			/* Yes, send UART data packet. */
			receive_flag = 0;
			usart_write_buffer_job(&usart_instance, tx_buffer, strlen((const char *)tx_buffer));
		}
		/*if the data has been received, blink the on board LED*/
		if (receive_flag == 1){
			port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
			delay_ms(200);
			port_pin_set_output_level(LED_0_PIN, !LED_0_ACTIVE);
			receive_flag = 0;
		}
	}
}



/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 * This code is for UART testing
 */

#include <asf.h>
#include <string.h>
#include "conf_clocks.h"
#include "delay.h"

#define UART_MODULE       SERCOM4
#define UART_SERCOM_MUX   USART_RX_3_TX_2_XCK_3
#define UART_SERCOM_GCLK  SERCOM4_GCLK_ID_CORE
#define BUFFER_SIZE 64

static uint8_t rx_buffer[BUFFER_SIZE];
static uint8_t tx_buffer[BUFFER_SIZE] = "UART!\r\n";

static struct usart_module usart_instance;
static int receive_flag = 0;

///////////////////////////
////UART
static void usart_read_callback(struct usart_module *const module) {
	// Reception completed
	usart_read_buffer_job(&usart_instance, rx_buffer, BUFFER_SIZE);
	if (port_pin_get_input_level(BUTTON_0_PIN) == !BUTTON_0_ACTIVE)
	{
	   receive_flag = 1;
	   }
	
}

static void configure_uart(void) {
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);

	config_usart.mux_setting = UART_SERCOM_MUX;
	config_usart.pinmux_pad0 = PINMUX_PB10D_SERCOM4_PAD2;
	config_usart.pinmux_pad1 = PINMUX_PB11D_SERCOM4_PAD3;

	while (usart_init(&usart_instance, UART_MODULE, &config_usart) != STATUS_OK) {}

	usart_enable(&usart_instance);

	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
	usart_register_callback(&usart_instance, usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_read_buffer_job(&usart_instance, rx_buffer, BUFFER_SIZE);
}

void configure_led(void) {
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	pin_conf.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_0_PIN, &pin_conf);
}

/////////////////////////////
int main (void)
{
	system_init();
	configure_led();
	delay_init();
	configure_uart();

	usart_write_buffer_job(&usart_instance, tx_buffer, strlen((const char *)tx_buffer));

	while (1) {
		/* Is button pressed? */
		if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
			/* Yes, send UART data packet. */
			usart_write_buffer_job(&usart_instance, tx_buffer, strlen((const char *)tx_buffer));
		}
		/*if the data has been received, blink the on board LED*/
		if (receive_flag == 1){
			port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
			delay_ms(200);
			port_pin_set_output_level(LED_0_PIN, !LED_0_ACTIVE);
			receive_flag = 0;
		}
	}
}






`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// insns are 32 bits in RV32IM
`define INSN_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`ifndef RISCV_FORMAL
`include "../hw2b/cla.sv"
`include "../hw3-singlecycle/RvDisassembler.sv"
`include "../hw4-multicycle/divider_unsigned_pipelined.sv"
`endif

module Disasm #(
    byte PREFIX = "D"
) (
    input wire [31:0] insn,
    output wire [(8*32)-1:0] disasm
);
  // synthesis translate_off
  // this code is only for simulation, not synthesis
  string disasm_string;
  always_comb begin
    disasm_string = rv_disasm(insn);
  end
  // HACK: get disasm_string to appear in GtkWave, which can apparently show only wire/logic. Also,
  // string needs to be reversed to render correctly.
  genvar i;
  for (i = 3; i < 32; i = i + 1) begin : gen_disasm
    assign disasm[((i+1-3)*8)-1-:8] = disasm_string[31-i];
  end
  assign disasm[255-:8] = PREFIX;
  assign disasm[247-:8] = ":";
  assign disasm[239-:8] = " ";
  // synthesis translate_on
endmodule

module RegFile (
    input logic [4:0] rd,
    input logic [`REG_SIZE] rd_data,
    input logic [4:0] rs1,
    output logic [`REG_SIZE] rs1_data,
    input logic [4:0] rs2,
    output logic [`REG_SIZE] rs2_data,

    input logic clk,
    input logic we,
    input logic rst
);
  localparam int NumRegs = 32;
  genvar i;
  logic [`REG_SIZE] regs[NumRegs];

  // TODO: your code here
  assign regs[0] = 32'd0; // x0 is always zero 
  assign rs1_data = regs[rs1]; // 1st read port 
  assign rs2_data = regs[rs2]; // 2nd read port
  for (i = 1; i < 32; i += 1) begin
    always_ff @(posedge clk) begin
      if (rst) begin 
        regs[i] <= 32'd0;
      end else begin
        if (we && rd == i) begin
          regs[rd] <= rd_data; 
        end
      end
    end
  end
endmodule
typedef enum {
  /** invalid value, this should never appear after the initial reset sequence completes */
  CYCLE_INVALID = 0,
  /** a stall cycle that arose from the initial reset signal */
  CYCLE_RESET = 1,
  /** not a stall cycle, a valid insn is in Writeback */
  CYCLE_NO_STALL = 2,
  /** a stall cycle that arose from a taken branch/jump */
  CYCLE_TAKEN_BRANCH = 4,

  // the values below are only needed in HW5B

  /** a stall cycle that arose from a load-to-use stall */
  CYCLE_LOAD2USE = 8,
  /** a stall cycle that arose from a div/rem-to-use stall */
  CYCLE_DIV2USE = 16,
  /** a stall cycle that arose from a fence.i insn */
  CYCLE_FENCEI = 32
} cycle_status_e;

/** state at the start of Decode stage */
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
} stage_decode_t;


module DatapathPipelined (
    input wire clk,
    input wire rst,
    output logic [`REG_SIZE] pc_to_imem,
    input wire [`INSN_SIZE] insn_from_imem,
    // dmem is read/write
    output logic [`REG_SIZE] addr_to_dmem,
    input wire [`REG_SIZE] load_data_from_dmem,
    output logic [`REG_SIZE] store_data_to_dmem,
    output logic [3:0] store_we_to_dmem,

    output logic halt,

    // The PC of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`REG_SIZE] trace_writeback_pc,
    // The bits of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`INSN_SIZE] trace_writeback_insn,
    // The status of the insn (or stall) currently in Writeback. See cycle_status_e enum for valid values.
    output cycle_status_e trace_writeback_cycle_status
);

  // opcodes - see section 19 of RiscV spec
  localparam bit [`OPCODE_SIZE] OpLoad = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpStore = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpBranch = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpJalr = 7'b11_001_11;
  localparam bit [`OPCODE_SIZE] OpMiscMem = 7'b00_011_11;
  localparam bit [`OPCODE_SIZE] OpJal = 7'b11_011_11;

  localparam bit [`OPCODE_SIZE] OpRegImm = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpRegReg = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpEnviron = 7'b11_100_11;

  localparam bit [`OPCODE_SIZE] OpAuipc = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpLui = 7'b01_101_11;

  // cycle counter, not really part of any stage but useful for orienting within GtkWave
  // do not rename this as the testbench uses this value
  logic [`REG_SIZE] cycles_current;
  always_ff @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
    end
  end
/**
 * This enum is used to classify each cycle as it comes through the Writeback stage, identifying
 * if a valid insn is present or, if it is a stall cycle instead, the reason for the stall. The
 * enum values are mutually exclusive: only one should be set for any given cycle. These values
 * are compared against the trace-*.json files to ensure that the datapath is running with the
 * correct timing.
 *
 * You will need to set these values at various places within your pipeline, and propagate them
 * through the stages until they reach Writeback where they can be checked.
 */
   /***************/
  /* FETCH STAGE */
  /***************/

  logic [`REG_SIZE] f_pc_current, pcNext;
  wire [`REG_SIZE] f_insn;
  cycle_status_e f_cycle_status, x_cycle_status;

  // program counter
  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc_current <= 32'd0;
      // NB: use CYCLE_NO_STALL since this is the value that will persist after the last reset cycle
      f_cycle_status <= CYCLE_NO_STALL;
    end else begin
      f_cycle_status <= CYCLE_NO_STALL;
      if (exec_state.cycle_status == CYCLE_TAKEN_BRANCH) begin
        f_pc_current <= pcNext;
      end else begin
        f_pc_current <= f_pc_current + 4;
      end
    end
  end
  // send PC to imem
  assign pc_to_imem = f_pc_current;
  assign f_insn = insn_from_imem;

  // Here's how to disassemble an insn into a string you can view in GtkWave.
  // Use PREFIX to provide a 1-character tag to identify which stage the insn comes from.
  Disasm #(
      .PREFIX("F")
  ) disasm_0fetch (
      .insn  (f_insn),
      .disasm()
  );

  /****************/
  /* DECODE STAGE */
  /****************/

  // this shows how to package up state in a `struct packed`, and how to pass it between stages
  stage_decode_t decode_state, exec_state, mem_state, wb_state;
  logic [31:0] rs1_data, rs2_data, rf_rs1_data, rf_rs2_data, rs1_data_temp, rs2_data_temp;
  logic [4:0] exec_rd, mem_rd, wb_rd; 

  always_ff @(posedge clk) begin
    if (rst) begin
      decode_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_RESET
      };
    end else begin
      begin
        rf_rs1_data <= rs1_data_temp;
        rf_rs2_data <= rs2_data_temp;
        decode_state <= '{
          pc: f_pc_current,
          insn: f_insn,
          cycle_status: f_cycle_status
        };
      end
    end
  end
  Disasm #(
      .PREFIX("D")
  ) disasm_1decode (
      .insn  (decode_state.insn),
      .disasm()
  );

  // TODO: your code here, though you will also need to modify some of the code above
  // TODO: the testbench requires that your register file instance is named `rf`

  wire [6:0] insn_funct7;
  wire [4:0] insn_rs2;
  wire [4:0] insn_rs1;
  wire [2:0] insn_funct3;
  wire [4:0] insn_rd;
  wire [`OPCODE_SIZE] insn_opcode;
    // split R-type instruction - see section 2.2 of RiscV spec

  RegFile rf(.rd(wb_rd), .rd_data(wb_rd_data), .rs1(decode_state.insn[19:15]), .rs1_data(rs1_data), .rs2(decode_state.insn[24:20]), .rs2_data(rs2_data), .clk(clk), .we(wb_we), .rst(rst));
  always @* begin
    if (wb_rd == decode_state.insn[19:15] && wb_rd != 0) begin
      rs1_data_temp = wb_rd_data;
    end else begin
      rs1_data_temp = rs1_data;
    end
    if (wb_rd == decode_state.insn[24:20] && wb_rd != 0) begin
      rs2_data_temp = wb_rd_data;
    end else begin
      rs2_data_temp = rs2_data;
    end
  end
  /*****************/
  /* EXECUTE STAGE */
  /*****************/
  assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = exec_state.insn;

  logic [31:0] exec_rs1_data, exec_rs2_data, exec_rd_data_out, mem_rd_data, wb_rd_data, adder1, adder2, adder_sum;
  logic exec_we, mem_we, wb_we, adder_cin;

  cla adder(.a(adder1), .b(adder2), .cin(adder_cin), .sum(adder_sum));
  
  always_ff @(posedge clk) begin
    if (rst) begin
      exec_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_RESET
      };
    end else begin
      exec_state <= '{
        pc: decode_state.pc,
        insn: decode_state.insn,
        cycle_status: x_cycle_status
      };
    end
  end

  // setup for I, S, B & J type instructions
  // I - short immediates and loads
  wire [11:0] imm_i;
  assign imm_i = exec_state.insn[31:20];
  wire [ 4:0] imm_shamt = exec_state.insn[24:20];

  // S - stores
  wire [11:0] imm_s;
  assign imm_s[11:5] = insn_funct7, imm_s[4:0] = insn_rd;

  // B - conditionals
  wire [12:0] imm_b;
  assign {imm_b[12], imm_b[10:5]} = insn_funct7, {imm_b[4:1], imm_b[11]} = insn_rd, imm_b[0] = 1'b0;

  // J - unconditional jumps
  wire [20:0] imm_j;
  assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {exec_state.insn[31:12], 1'b0};

  wire [`REG_SIZE] imm_i_sext = {{20{imm_i[11]}}, imm_i[11:0]};
  wire [`REG_SIZE] imm_s_sext = {{20{imm_s[11]}}, imm_s[11:0]};
  wire [`REG_SIZE] imm_b_sext = {{19{imm_b[12]}}, imm_b[12:0]};
  wire [`REG_SIZE] imm_j_sext = {{11{imm_j[20]}}, imm_j[20:0]};

  wire insn_lui = insn_opcode == OpLui;
  wire insn_auipc = insn_opcode == OpAuipc;
  wire insn_jal = insn_opcode == OpJal;
  wire insn_jalr = insn_opcode == OpJalr;

  wire insn_beq = insn_opcode == OpBranch && exec_state.insn[14:12] == 3'b000;
  wire insn_bne = insn_opcode == OpBranch && exec_state.insn[14:12] == 3'b001;
  wire insn_blt = insn_opcode == OpBranch && exec_state.insn[14:12] == 3'b100;
  wire insn_bge = insn_opcode == OpBranch && exec_state.insn[14:12] == 3'b101;
  wire insn_bltu = insn_opcode == OpBranch && exec_state.insn[14:12] == 3'b110;
  wire insn_bgeu = insn_opcode == OpBranch && exec_state.insn[14:12] == 3'b111;

  wire insn_lb = insn_opcode == OpLoad && exec_state.insn[14:12] == 3'b000;
  wire insn_lh = insn_opcode == OpLoad && exec_state.insn[14:12] == 3'b001;
  wire insn_lw = insn_opcode == OpLoad && exec_state.insn[14:12] == 3'b010;
  wire insn_lbu = insn_opcode == OpLoad && exec_state.insn[14:12] == 3'b100;
  wire insn_lhu = insn_opcode == OpLoad && exec_state.insn[14:12] == 3'b101;

  wire insn_sb = insn_opcode == OpStore && exec_state.insn[14:12] == 3'b000;
  wire insn_sh = insn_opcode == OpStore && exec_state.insn[14:12] == 3'b001;
  wire insn_sw = insn_opcode == OpStore && exec_state.insn[14:12] == 3'b010;

  wire insn_addi = insn_opcode == OpRegImm && exec_state.insn[14:12] == 3'b000;
  wire insn_slti = insn_opcode == OpRegImm && exec_state.insn[14:12] == 3'b010;
  wire insn_sltiu = insn_opcode == OpRegImm && exec_state.insn[14:12] == 3'b011;
  wire insn_xori = insn_opcode == OpRegImm && exec_state.insn[14:12] == 3'b100;
  wire insn_ori = insn_opcode == OpRegImm && exec_state.insn[14:12] == 3'b110;
  wire insn_andi = insn_opcode == OpRegImm && exec_state.insn[14:12] == 3'b111;

  wire insn_slli = insn_opcode == OpRegImm && exec_state.insn[14:12] == 3'b001 && exec_state.insn[31:25] == 7'd0;
  wire insn_srli = insn_opcode == OpRegImm && exec_state.insn[14:12] == 3'b101 && exec_state.insn[31:25] == 7'd0;
  wire insn_srai = insn_opcode == OpRegImm && exec_state.insn[14:12] == 3'b101 && exec_state.insn[31:25] == 7'b0100000;

  wire insn_add = insn_opcode == OpRegReg && exec_state.insn[14:12] == 3'b000 && exec_state.insn[31:25] == 7'd0;
  wire insn_sub  = insn_opcode == OpRegReg && exec_state.insn[14:12] == 3'b000 && exec_state.insn[31:25] == 7'b0100000;
  wire insn_sll = insn_opcode == OpRegReg && exec_state.insn[14:12] == 3'b001 && exec_state.insn[31:25] == 7'd0;
  wire insn_slt = insn_opcode == OpRegReg && exec_state.insn[14:12] == 3'b010 && exec_state.insn[31:25] == 7'd0;
  wire insn_sltu = insn_opcode == OpRegReg && exec_state.insn[14:12] == 3'b011 && exec_state.insn[31:25] == 7'd0;
  wire insn_xor = insn_opcode == OpRegReg && exec_state.insn[14:12] == 3'b100 && exec_state.insn[31:25] == 7'd0;
  wire insn_srl = insn_opcode == OpRegReg && exec_state.insn[14:12] == 3'b101 && exec_state.insn[31:25] == 7'd0;
  wire insn_sra  = insn_opcode == OpRegReg && exec_state.insn[14:12] == 3'b101 && exec_state.insn[31:25] == 7'b0100000;
  wire insn_or = insn_opcode == OpRegReg && exec_state.insn[14:12] == 3'b110 && exec_state.insn[31:25] == 7'd0;
  wire insn_and = insn_opcode == OpRegReg && exec_state.insn[14:12] == 3'b111 && exec_state.insn[31:25] == 7'd0;

  wire insn_mul    = insn_opcode == OpRegReg && exec_state.insn[31:25] == 7'd1 && exec_state.insn[14:12] == 3'b000;
  wire insn_mulh   = insn_opcode == OpRegReg && exec_state.insn[31:25] == 7'd1 && exec_state.insn[14:12] == 3'b001;
  wire insn_mulhsu = insn_opcode == OpRegReg && exec_state.insn[31:25] == 7'd1 && exec_state.insn[14:12] == 3'b010;
  wire insn_mulhu  = insn_opcode == OpRegReg && exec_state.insn[31:25] == 7'd1 && exec_state.insn[14:12] == 3'b011;
  wire insn_div    = insn_opcode == OpRegReg && exec_state.insn[31:25] == 7'd1 && exec_state.insn[14:12] == 3'b100;
  wire insn_divu   = insn_opcode == OpRegReg && exec_state.insn[31:25] == 7'd1 && exec_state.insn[14:12] == 3'b101;
  wire insn_rem    = insn_opcode == OpRegReg && exec_state.insn[31:25] == 7'd1 && exec_state.insn[14:12] == 3'b110;
  wire insn_remu   = insn_opcode == OpRegReg && exec_state.insn[31:25] == 7'd1 && exec_state.insn[14:12] == 3'b111;

  wire insn_ecall = insn_opcode == OpEnviron && exec_state.insn[31:7] == 25'd0;
  wire insn_fence = insn_opcode == OpMiscMem;
  always_latch begin
    x_cycle_status = decode_state.cycle_status;
    exec_rs1_data = rf_rs1_data;
    exec_rs2_data = rf_rs2_data;
    
    if (mem_rd == insn_rs1 && mem_rd != 0) begin
      exec_rs1_data = mem_rd_data;
    end else if (wb_rd == insn_rs1 && wb_rd != 0) begin
      exec_rs1_data = wb_rd_data;
    end
    if (mem_rd == insn_rs2 && mem_rd != 0) begin
      exec_rs2_data = mem_rd_data;
    end else if (wb_rd == insn_rs2 && wb_rd != 0) begin
      exec_rs2_data = wb_rd_data;
    end

    case (insn_opcode)
      OpLui: begin
        exec_we = 1;
        exec_rd_data_out = {exec_state.insn[31:12], 12'b0};
      end
      OpRegImm: begin
        exec_we = 1;
        if (insn_addi) begin
          if (mem_rd == insn_rs1 && mem_rd != 0) begin
            adder1 = mem_rd_data;
          end else if (wb_rd == insn_rs1 && wb_rd != 0) begin
            adder1 = wb_rd_data;
          end else begin
            adder1 = rf_rs1_data;
          end
          adder2 = imm_i_sext;
          adder_cin = 0;
          exec_rd_data_out = adder_sum;
        end else if (insn_slti) begin
          if ($signed(exec_rs1_data) < $signed(imm_i_sext)) begin
            exec_rd_data_out = 1;
          end else begin
            exec_rd_data_out = 0;
          end
        end else if (insn_sltiu) begin // sltiu
          if (exec_rs1_data < imm_i_sext) begin
            exec_rd_data_out = 1;
          end else begin
            exec_rd_data_out = 0;
          end
        end else if (insn_xori) begin // xori
          exec_rd_data_out = exec_rs1_data ^ imm_i_sext;
        end else if (insn_ori) begin // ori
          exec_rd_data_out = exec_rs1_data | imm_i_sext;
        end else if (insn_andi) begin // andi
          exec_rd_data_out = exec_rs1_data & imm_i_sext;
        end else if (insn_slli) begin
          exec_rd_data_out = exec_rs1_data << imm_i[4:0];
        end else if (insn_srli) begin
          exec_rd_data_out = exec_rs1_data >> imm_i[4:0];
        end else if (insn_srai) begin
          exec_rd_data_out = $signed(exec_rs1_data) >>> imm_i[4:0];
        end else begin
          exec_rd_data_out = 0;
          x_cycle_status = CYCLE_INVALID;
        end
      end
      OpRegReg: begin
        exec_we = 1;
        if (insn_add) begin
          if (mem_rd == insn_rs1 && mem_rd != 0) begin
            adder1 = mem_rd_data;
          end else if (wb_rd == insn_rs1 && wb_rd != 0) begin
            adder1 = wb_rd_data;
          end else begin
            adder1 = rf_rs1_data;
          end
          if (mem_rd == insn_rs2 && mem_rd != 0) begin
            adder2 = mem_rd_data;
          end else if (wb_rd == insn_rs2 && wb_rd != 0) begin
            adder2 = wb_rd_data;
          end else begin
            adder2 = rf_rs2_data;
          end
          adder_cin = 0;
          exec_rd_data_out = adder_sum;
        end
      end
      OpBranch: begin
        exec_we = 0;
        if (insn_beq) begin
          if (exec_rs1_data == exec_rs2_data) begin
            x_cycle_status = CYCLE_TAKEN_BRANCH;
            pcNext = exec_state.pc + imm_b_sext;
          end
        end else if (insn_bne) begin
          if (exec_rs1_data != exec_rs2_data) begin
            x_cycle_status = CYCLE_TAKEN_BRANCH;
            pcNext = exec_state.pc + imm_b_sext;
          end
        end
      end
      OpEnviron: begin
        if (exec_state.insn[31:7] == 25'd0) begin
          //halt = 1'b1;
        end else begin

        end
      end
      default: begin
        x_cycle_status = CYCLE_INVALID;
      end
    endcase
  end
  
  /****************/
  /* MEMORY STAGE */
  /****************/
   logic [31:0] temp;
  logic [31:0] addr, store_data;
  logic [3:0] store_we;
  assign addr_to_dmem = addr;
  assign store_data_to_dmem = store_data;
  assign store_we_to_dmem = store_we;

  always_ff @(posedge clk) begin
    if (rst) begin
      mem_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_RESET
      };
      mem_we <= 0;
    end else begin
      mem_state <= exec_state;
      mem_we <= exec_we;
      mem_rd_data <= exec_rd_data_out;
      mem_rd <= insn_rd;
    end
  end
  
  /*******************/
  /* WRITEBACK STAGE */
  /*******************/

  always_ff @(posedge clk) begin
    if (rst) begin
      wb_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_RESET
      };
      wb_we <= 0;
    end else begin
      wb_state <= mem_state;
      wb_we <= mem_we;
      wb_rd_data <= mem_rd_data;
      wb_rd <= mem_rd;
    end
  end
  
  always_comb begin
    halt = 0;
    if (wb_state.insn[6:0] == OpEnviron) begin
      halt = 1;
    end
  end

  assign trace_writeback_pc = wb_state.pc;
  assign trace_writeback_insn = wb_state.insn;
  assign trace_writeback_cycle_status = wb_state.cycle_status;

endmodule

module MemorySingleCycle #(
    parameter int NUM_WORDS = 512
) (
    // rst for both imem and dmem
    input wire rst,

    // clock for both imem and dmem. The memory reads/writes on @(negedge clk)
    input wire clk,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] pc_to_imem,

    // the value at memory location pc_to_imem
    output logic [`REG_SIZE] insn_from_imem,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] addr_to_dmem,

    // the value at memory location addr_to_dmem
    output logic [`REG_SIZE] load_data_from_dmem,

    // the value to be written to addr_to_dmem, controlled by store_we_to_dmem
    input wire [`REG_SIZE] store_data_to_dmem,

    // Each bit determines whether to write the corresponding byte of store_data_to_dmem to memory location addr_to_dmem.
    // E.g., 4'b1111 will write 4 bytes. 4'b0001 will write only the least-significant byte.
    input wire [3:0] store_we_to_dmem
);

  // memory is arranged as an array of 4B words
  logic [`REG_SIZE] mem[NUM_WORDS];

  initial begin
    $readmemh("mem_initial_contents.hex", mem, 0);
  end

  always_comb begin
    // memory addresses should always be 4B-aligned
    assert (pc_to_imem[1:0] == 2'b00);
    assert (addr_to_dmem[1:0] == 2'b00);
  end

  localparam int AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam int AddrLsb = 2;

  always @(negedge clk) begin
    if (rst) begin
    end else begin
      insn_from_imem <= mem[{pc_to_imem[AddrMsb:AddrLsb]}];
    end
  end
   always @(negedge clk) begin
    if (rst) begin
    end else begin
      if (store_we_to_dmem[0]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
      end
      if (store_we_to_dmem[1]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
      end
      if (store_we_to_dmem[2]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
      end
      if (store_we_to_dmem[3]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
      end
      // dmem is "read-first": read returns value before the write
      load_data_from_dmem <= mem[{addr_to_dmem[AddrMsb:AddrLsb]}];
    end
  end
endmodule

/* This design has just one clock for both processor and memory. */
module RiscvProcessor (
    input  wire  clk,
    input  wire  rst,
    output logic halt,
    output wire [`REG_SIZE] trace_writeback_pc,
    output wire [`INSN_SIZE] trace_writeback_insn,
    output cycle_status_e trace_writeback_cycle_status
);

  wire [`INSN_SIZE] insn_from_imem;
  wire [`REG_SIZE] pc_to_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [3:0] mem_data_we;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) mem (
      .rst                (rst),
      .clk                (clk),
      // imem is read-only
      .pc_to_imem         (pc_to_imem),
      .insn_from_imem     (insn_from_imem),
      // dmem is read-write
      .addr_to_dmem       (mem_data_addr),
      .load_data_from_dmem(mem_data_loaded_value),
      .store_data_to_dmem (mem_data_to_write),
      .store_we_to_dmem   (mem_data_we)
  );

  DatapathPipelined datapath (
      .clk(clk),
      .rst(rst),
      .pc_to_imem(pc_to_imem),
      .insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),
      .store_data_to_dmem(mem_data_to_write),
      .store_we_to_dmem(mem_data_we),
      .load_data_from_dmem(mem_data_loaded_value),
      .halt(halt),
      .trace_writeback_pc(trace_writeback_pc),
      .trace_writeback_insn(trace_writeback_insn),
      .trace_writeback_cycle_status(trace_writeback_cycle_status)
  );

endmodule



