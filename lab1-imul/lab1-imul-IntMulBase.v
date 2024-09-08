//========================================================================
// Integer Multiplier Fixed-Latency Implementation
//========================================================================

`ifndef LAB1_IMUL_INT_MUL_BASE_V
`define LAB1_IMUL_INT_MUL_BASE_V

`include "lab1-imul-msgs.v"
`include "vc-trace.v"
`include "vc-muxes.v" 
`include "vc-regs.v" 
`include "vc-arithmetic.v"

// Define datapath and control unit here

//========================================================================
// Integer Multiplier Fixed-Latency Implementation
//========================================================================

module lab1_imul_IntMulBase
(
  input  logic                clk,
  input  logic                reset,

  input  logic                req_val,
  output logic                req_rdy,
  input  lab1_imul_req_msg_t  req_msg,

  output logic                resp_val,
  input  logic                resp_rdy,
  output lab1_imul_resp_msg_t resp_msg
);

  //----------------------------------------------------------------------
  // Trace request message
  //----------------------------------------------------------------------

  lab1_imul_ReqMsgTrace req_msg_trace
  (
    .clk   (clk),
    .reset (reset),
    .val   (req_val),
    .rdy   (req_rdy),
    .msg   (req_msg)
  );

  // Instantiate datapath and control models here and then connect them
  // together. As a place holder, for now we simply pass input operand
  // A through to the output, which obviously is not correct.

 



  // assign req_rdy         = resp_rdy;
  // assign resp_val        = req_val;
  // assign resp_msg.result = req_msg.a;

  //----------------------------------------------------------------------
  // Line Tracing
  //----------------------------------------------------------------------

  `ifndef SYNTHESIS

  reg [`VC_TRACE_NBITS_TO_NCHARS(32)*8-1:0] str;

  `VC_TRACE_BEGIN
  begin

    req_msg_trace.trace( trace_str );

    vc_trace.append_str( trace_str, "(" );

    // Add extra line tracing for internal state here

    vc_trace.append_str( trace_str, ")" );

    $sformat( str, "%x", resp_msg );
    vc_trace.append_val_rdy_str( trace_str, resp_val, resp_rdy, str );

  end
  `VC_TRACE_END

  `endif /* SYNTHESIS */

endmodule

`endif /* LAB1_IMUL_INT_MUL_BASE_V */




module DataPath (input clk, b_mux_sel, a_mux_sel, result_mux_sel, result_en, add_mux_sel, 
                 input [31:0] req_msg_a, req_msg_b,
                 output b_lsb, output [31:0]resp_msg );  

localparam nbits = 32;

///******************  b reg  *******************////
wire [nbits-1:0] b_reg_in, b_reg_out;
vc_Reg #(.p_nbits(nbits)) b_reg (.clk(clk), .q(b_reg_out), .d(b_reg_in));

wire [nbits-1:0] b_reg_shift_out;
vc_Mux2 #(.p_nbits(nbits)) b_mux (.in0(b_reg_shift_out), .in1(req_msg_b), .sel(b_mux_sel), .out(b_reg_in));

RightShift #(.p_nbits(nbits)) b_RightShift (.in(b_reg_out), .out(b_reg_shift_out));

assign b_lsb = b_reg_out[0];

///******************  a reg  *******************////
wire [nbits-1:0] a_reg_in, a_reg_out;
vc_Reg #(.p_nbits(nbits)) a_reg (.clk(clk), .q(a_reg_out), .d(a_reg_in));

wire [nbits-1:0] a_reg_shift_out;
vc_Mux2 #(.p_nbits(nbits)) a_mux (.in0(a_reg_shift_out), .in1(req_msg_a), .sel(a_mux_sel), .out(a_reg_in));

LeftShift #(.p_nbits(nbits)) a_LeftShift (.in(a_reg_out), .out(a_reg_shift_out));


///******************  result reg  *******************////
wire [nbits-1:0] result_reg_in, result_reg_out;
vc_EnReg #(.p_nbits(nbits)) result_reg(.clk(clk), .reset(1'b0), .q(result_reg_out), .d(result_reg_in), .en(result_en));

wire [nbits-1:0] add_mux_out;
vc_Mux2 #(.p_nbits(nbits)) result_mux(.in0(add_mux_out), .in1(32'b0), .sel(result_mux_sel), .out(result_reg_in));

wire [nbits-1:0] result_adder_out;
vc_SimpleAdder #(.p_nbits(nbits)) (.in0(a_reg_out), .in1(result_reg_out), .out(result_adder_out));

vc_Mux2 #(.p_nbits(nbits)) add_mux(.in0(result_adder_out), in1(result_reg_out), .sel(add_mux_sel), .out(add_mux_out));

assign resp_msg = result_reg_out;

endmodule 



//// RightShift  
module RightShift #(p_nbits = 1) (input [p_nbits-1:0] in, output [p_nbits-1:0] out);
assign out = in >> 1;
endmodule


//// LeftShift 
module LeftShift #(p_nbits = 1) (input [p_nbits-1:0] in, output [p_nbits-1:0] out);
assign out = in << 1;
endmodule