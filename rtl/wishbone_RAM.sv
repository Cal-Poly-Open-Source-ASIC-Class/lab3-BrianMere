`timescale 1ns/1ps

`define RAM_INPUT_WE(bus_4, we) (bus_4 & {4{we}})

module wishbone_RAM (

    // Both Ports
    input logic clk,         // Sync. Clock
    input logic rst,         // Sync. Reset

    // Port A (wishbone)
    input logic [10:0] pA_wb_addr_i,// address line
    input logic [31:0] pA_wb_data_i,// data line in
    output logic [31:0] pA_wb_data_o,// data line out
    input logic pA_wb_we_i,  // write enable, or reads on low
    input logic [3:0] pA_wb_sel_i, // byte select, for single byte selection

    input logic pA_wb_stb_i, // valid transfer signal, starts transaction
    output logic pA_wb_ack_o,   // terminates normal bus cycle, indicating end of transaction
    input logic pA_wb_cyc_i, // held high during the whole transaction
    output logic pA_wb_stall_o, // hold high if this port needs to stall on a mem read

    // Port B (wishbone)
    input logic [10:0] pB_wb_addr_i,// address line
    input logic [31:0] pB_wb_data_i,// data line in
    output logic [31:0] pB_wb_data_o,// data line out
    input logic pB_wb_we_i,  // write enable, or reads on low
    input logic [3:0] pB_wb_sel_i, // byte select, for single byte selection

    input logic pB_wb_stb_i, // valid transfer signal, qualifies addr, write enable, ...
    output logic pB_wb_ack_o,   // terminates normal bus cycle, indicating end of transaction
    input logic pB_wb_cyc_i, // held high during the whole transaction
    output logic pB_wb_stall_o // hold high if this port needs to stall on a mem read
);

    typedef struct packed {
        logic [3:0] we;
        logic [31:0] din;
        logic [7:0] addr;
    } RAM_Input_t;

    RAM_Input_t sel_0, sel_1;
    logic [31:0] dout_0, dout_1;

    // Instantiate our modules
    DFFRAM256x32 RAM0(
        .CLK(clk), 
        .WE0(sel_0.we), // write enable
        .EN0(1), // read enable (outputs 0 otherwise)
        .Di0(sel_0.din), // Data in
        .Do0(dout_0), 
        .A0(sel_0.addr)
    );
    DFFRAM256x32 RAM1(
        .CLK(clk), 
        .WE0(sel_1.we), // write enable
        .EN0(1), // read enable (outputs 0 otherwise)
        .Di0(sel_1.din), // Data in
        .Do0(dout_1), 
        .A0(sel_1.addr)
    );

    always_comb begin : InputStructHandling // handle the math for the input struct wires, along with muxing
        // For the we, we want to select the bits, as well as use the write_enable input
        if(sel0_useA) begin 
            sel_0.we = `RAM_INPUT_WE(pA_wb_sel_i, pA_wb_we_i);
            sel_1.we = `RAM_INPUT_WE(pB_wb_sel_i, pB_wb_we_i);
        end else begin 
            sel_0.we = `RAM_INPUT_WE(pB_wb_sel_i, pB_wb_we_i);
            sel_1.we = `RAM_INPUT_WE(pA_wb_sel_i, pA_wb_we_i);
        end

        // Data in just is passed through
        if(sel0_useA) begin
            sel_0.din = pA_wb_data_i;
            sel_1.din = pB_wb_data_i;
        end else begin
            sel_0.din = pB_wb_data_i;
            sel_1.din = pA_wb_data_i;
        end

        // Address is word addressable, so just get the nearest word
        if(sel0_useA) begin 
            sel_0.addr = pA_wb_addr_i[9:2];
            sel_1.addr = pB_wb_addr_i[9:2];
        end else begin 
            sel_0.addr = pB_wb_addr_i[9:2];
            sel_1.addr = pA_wb_addr_i[9:2];
        end

        // ... and from the address get the RAM selection via the top bit
        a_ramsel = pA_wb_addr_i[10];
        b_ramsel = pB_wb_addr_i[10];
    end

    // DEFINE what logic we need from the CD here...

    // input: Selectors of what ram the port<n> WANTS to use
    logic a_ramsel, b_ramsel;
    
    // output: these are high when the associated RAM<n> should use portA. Otherwise use B. 
    logic sel0_useA;
    // and make sure to register all outputs of the CD
    logic sel0_useA_1;
    always_ff @( posedge clk ) begin 
        if(rst) begin
            sel0_useA_1 <= 0;
        end
        else begin 
            sel0_useA_1 <= sel0_useA;
        end
    end

    // END DEFINE
    collision_detector CD(.*);

    // Given dout_0 and dout_1, using the staggered sel0_useA... values get the output
    // data to the right place
    always_comb begin : OutputDataHandling
        if(sel0_useA_1) begin 
            pA_wb_data_o = dout_0;
            pB_wb_data_o = dout_1;
        end else begin // you could use the other wire but they are always inverses
            pA_wb_data_o = dout_1;
            pB_wb_data_o = dout_0;
        end
    end
    
endmodule