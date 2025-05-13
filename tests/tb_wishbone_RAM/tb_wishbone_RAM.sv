`timescale 1ns/1ps

module tb_wishbone_RAM;

`ifdef USE_POWER_PINS
    wire VPWR;
    wire VGND;
    assign VPWR=1;
    assign VGND=0;
`endif

logic clk = 0;        // Sync. Clock
logic rst = 0;        // Sync. Reset

// Port A (wishbone)
logic [10:0] pA_wb_addr_i=0;// address line
logic [31:0] pA_wb_data_i=0;// data line out
logic [31:0] pA_wb_data_o;// data line out
logic pA_wb_we_i=0;  // write enable, or reads on low
logic [3:0] pA_wb_sel_i=0; // byte select, for single byte selection

logic pA_wb_stb_i=0; // valid transfer signal, starts transaction
logic pA_wb_ack_o;   // terminates normal bus cycle, indicating end of transaction
logic pA_wb_cyc_i=0; // held high during the whole transaction
logic pA_wb_stall_o; // hold high if this port needs to stall on a mem read

// Port B (wishbone)
logic [10:0] pB_wb_addr_i=0;// address line
logic [31:0] pB_wb_data_i=0;// data line out
logic [31:0] pB_wb_data_o;// data line out
logic pB_wb_we_i=0;  // write enable, or reads on low
logic [3:0] pB_wb_sel_i=0; // byte select, for single byte selection

logic pB_wb_stb_i=0; // valid transfer signal, qualifies addr, write enable, ...
logic pB_wb_ack_o;   // terminates normal bus cycle, indicating end of transaction
logic pB_wb_cyc_i=0; // held high during the whole transaction
logic pB_wb_stall_o; // hold high if this port needs to stall on a mem read
    
wishbone_RAM DUT(.*);

// Necessary to create Waveform
initial begin
    // Name as needed
    $dumpfile("tb_async_fifo.vcd");
    $dumpvars(0);
end

// Random seeded number generator for tests
function automatic logic [63:0] lsfr(input logic [63:0] current);
  logic feedback;
  feedback = current[63] ^ current[62] ^ current[60] ^ current[59];
  return {current[62:0], feedback};
endfunction
logic [63:0] lfsr_v;
initial begin
  lfsr_v = 64'hDEADBEEFCAFEBABE; // seed — must not be 0
end
task automatic random_switch(output logic [63:0] out);
  lfsr_v = lsfr(lfsr_v);
  out = lfsr_v;
endtask
function automatic logic [63:0] random();
    logic [63:0] out;
    random_switch(out);
    return out;
endfunction

localparam TIME_PERIOD = 10;
initial begin
    forever begin 
        #(TIME_PERIOD/2);
        clk=~clk;
    end
end

logic[12:0] addrA, addrB;
logic[31:0] dataA, dataB;

logic[7:0] debug_dataA, debug_dataB;

localparam NUM_TESTS = 100;
always begin : MainTB
    resetTest();
    for(int i = 1; i != NUM_TESTS; i++) begin 
        $display("Testing %d of %d", i, NUM_TESTS);

        // A alone, B alone
        generateValues();
        addrB = ~addrA; // to ensure they are in different address spaces. Flip that top bit. 
        $display("Write: [%h] => A@0x%h, [%h] => B@0x%h", dataA[7:0], addrA[12:0], dataB[7:0], addrB[12:0]);
        WriteTest(addrA[12:0], addrB[12:0], dataA[7:0], dataB[7:0], 0, 1);

        // A alone, B alone (words)
        addrA >>= 2;
        addrB >>= 2;
        $display("Words: [%h] => A@0x%h, [%h] => B@0x%h", dataA, addrA[10:0], dataB, addrB[10:0]);
        WriteWordTest(addrA[10:0], addrB[10:0], dataA, dataB);

        // A, B together (stall)
        addrA = addrB;
        $display("Stall: [%h] => A@0x%h, [%h] => B@0x%h",, dataA[7:0], addrA[12:0], dataB[7:0], addrB[12:0]);
        WriteTest(addrA[12:0], addrB[12:0], dataA[7:0], dataB[7:0], 1, 1);
        $display("Stall w/ Words: [%h] => A@0x%h, [%h] => B@0x%h", dataA, addrA[12:0], dataB, addrB[12:0]);
        WriteWordTest(addrA[10:0], addrB[10:0], dataA, dataB);
    end

    $finish;
end

/** generate values for the addresses and data's */
task generateValues();
    addrA = random()[12:0];
    addrB = random()[12:0];
    dataA = random()[31:0];
    dataB = random()[31:0];
    debug_dataA = dataA[7:0];
    debug_dataB = dataB[7:0];
endtask

/**
    Do a reset test.
*/
task resetTest();
    rst = 1;
    #TIME_PERIOD;
    assert(pA_wb_ack_o == 0 && pB_wb_ack_o == 0) else $error("ACK was set high on reset.");
    assert(pA_wb_data_o == 0 && pB_wb_data_o == 0) else $error("Internal memory module was not reset!");
    assert(pA_wb_stall_o == 0 && pB_wb_stall_o == 0) else $error("Stall happened on a reset?");
    rst = 0;
    #TIME_PERIOD;
endtask

/** Try to read data at `raddr`. 
    Assign via port A if `use_A` is asserted. Else, use B. 

    This doesn't do the clocking, but instead sets up the signals such that a read on 
    this port is done. 
*/
task readInput(input logic[12:0] raddr, input logic use_A = 1);
    if(use_A) begin 
        pA_wb_addr_i = raddr[12:2];
        pA_wb_data_i = 32'hDEADBEEF; // this shouldn't ever be used!
        pA_wb_we_i   = 0;
        pA_wb_sel_i  = 1 << raddr[1:0];
        pA_wb_stb_i  = 1;
        pA_wb_cyc_i  = 1;
    end else begin 
        pB_wb_addr_i = raddr[12:2];
        pB_wb_data_i = 32'hDEADBEEF; // this shouldn't ever be used!
        pB_wb_we_i   = 0;
        pB_wb_sel_i  = 1 << raddr[1:0];
        pB_wb_stb_i  = 1;
        pB_wb_cyc_i  = 1;
    end
endtask 

/** End any readInput()/writeInput() with a corresponding endReadInput(), for the sake of the stb. 
    NO CLOCKING
*/
task endInput(input logic use_A = 1);
    if(use_A) pA_wb_stb_i = 0; else pB_wb_stb_i = 0;
endtask

/** End any asserts/reads of outputs with this line */
task endOutput(input logic use_A = 1);
    if(use_A) pA_wb_cyc_i = 0; else pB_wb_cyc_i = 0;
endtask

/** Try to write data at `waddr` with the corresponding `wdata`. 
    Assign via port A if `use_A` is asserted. Else, use B. 

    NO CLOCKING
*/
task writeInput(input logic[12:0] waddr, input logic[7:0] wdata, input logic use_A = 1);
    if(use_A) begin 
        pA_wb_addr_i = waddr[12:2];
        pA_wb_data_i = {24'b0, wdata};
        pA_wb_we_i   = 1;
        pA_wb_sel_i  = 1 << waddr[1:0];
        pA_wb_stb_i  = 1;
        pA_wb_cyc_i  = 1;
    end else begin 
        pB_wb_addr_i = waddr[12:2];
        pB_wb_data_i = {24'b0, wdata};
        pB_wb_we_i   = 1;
        pB_wb_sel_i  = 1 << waddr[1:0];
        pB_wb_stb_i  = 1;
        pB_wb_cyc_i  = 1;
    end
endtask

/** Try to read data at `raddr` */
task readInputWord(input logic[10:0] raddr, input logic use_A = 1);
    readInput(raddr << 2, use_A);
    if(use_A) pA_wb_sel_i = 4'b1111; else pB_wb_sel_i = 4'b1111;
endtask

/** Try to write data at `waddr` with word `wdata` */
task writeInputWord(input logic[10:0] waddr, input logic[31:0] wdata, input logic use_A = 1); 
    if(use_A) begin 
        pA_wb_addr_i = waddr;
        pA_wb_data_i = wdata;
        pA_wb_we_i   = 1;
        pA_wb_sel_i  = 4'b1111;
        pA_wb_stb_i  = 1;
        pA_wb_cyc_i  = 1;
    end else begin 
        pB_wb_addr_i = waddr;
        pB_wb_data_i = wdata;
        pB_wb_we_i   = 1;
        pB_wb_sel_i  = 4'b1111;
        pB_wb_stb_i  = 1;
        pB_wb_cyc_i  = 1;
    end
endtask

/** Tests reads on separate ports (clocking in between with `clock=0` or not `clock=1`), asserting with `rdataA`, ... */
task automatic ReadTest(input logic[12:0] raddrA, input logic[12:0] raddrB, input logic[7:0] rdataA, input logic[7:0] rdataB, input logic clock = 1,
    input logic do_asserts = 1);

    if(clock) begin 
        readInput(raddrA, 1);
        readInput(raddrB, 0);
        #TIME_PERIOD;
        endInput(1);
        endInput(0);
        if(do_asserts) assert({24'b0, rdataA} == pA_wb_data_o) else $error("A: Read data was expected %h but got %h", {24'b0, rdataA}, pA_wb_data_o);
        if(do_asserts) assert({24'b0, rdataB} == pB_wb_data_o) else $error("B: Read data was expected %h but got %h", {24'b0, rdataB}, pB_wb_data_o);
    end else begin 
        // A case:
        readInput(raddrA, 1);
        #TIME_PERIOD;
        endInput(1);
        if(do_asserts) assert({24'b0, rdataA} == pA_wb_data_o) else $error("A: Read data was expected %h but got %h", {24'b0, rdataA}, pA_wb_data_o);

        // B case:
        readInput(raddrB, 0);
        #TIME_PERIOD;
        endInput(0);
        if(do_asserts) assert({24'b0, rdataB} == pB_wb_data_o) else $error("B: Read data was expected %h but got %h", {24'b0, rdataB}, pB_wb_data_o);
    end
    endOutput(1);
    endOutput(0);
endtask

/** Tests writes on separate ports (clocking in between with `clock=0` or not `clock=1`), asserting with `rdataA`, ... */
task automatic WriteTest(input logic[12:0] waddrA, input logic[12:0] waddrB, input logic[7:0] wdataA, input logic[7:0] wdataB, input logic clock = 1,
    input logic do_asserts = 1);

    if(clock) begin 
        writeInput(waddrA, wdataA, 1);
        writeInput(waddrB, wdataB, 0);
        #TIME_PERIOD;
        endInput(1);
        endInput(0);
        $display("Write test:::");
        ReadTest(waddrA, waddrB, wdataA, wdataB, clock, do_asserts);
    end else begin 
        // A case:
        writeInput(waddrA, 1);
        #TIME_PERIOD;
        endInput(1);
        $display("Write test:::");
        ReadTest(waddrA, waddrB, wdataA, wdataB, clock, do_asserts);

        // B case:
        readInput(waddrB, 0);
        #TIME_PERIOD;
        endInput(0);
        $display("Write test:::");
        ReadTest(waddrA, waddrB, wdataA, wdataB, clock, do_asserts);
    end
    endOutput(1);
    endOutput(0);
endtask

/** Tests reads on separate ports (see soleReadTest()) but now by word, which must clock together here (assume you did more granular tests like Read/WriteTest) */
task automatic ReadWordTest(input logic[10:0] raddrA, input logic[10:0] raddrB, input logic [31:0] rdataA, input logic[31:0] rdataB);
    ReadTest(raddrA<<2, raddrB<<2, 0,0, 1, 0); // do a ReadTest with no asserts
    assert(pA_wb_data_o == rdataA) else $error("A: Tried reading word got %h expected %h", pA_wb_data_o, rdataA);
    assert(pB_wb_data_o == rdataB) else $error("B: Tried reading word got %h expected %h", pB_wb_data_o, rdataB);
    endOutput(1);
    endOutput(0);
endtask

/** Tests writes on separate ports (see soleReadTest()) but now by word, which must clock together here (assume you did more granular tests like Read/WriteTest) */
task automatic WriteWordTest(input logic[10:0] waddrA, input logic[10:0] waddrB, input logic [31:0] wdataA, input logic[31:0] wdataB);
    WriteTest(waddrA<<2, waddrB<<2, 0,0, 1, 0); // do a ReadTest with no asserts
    $display("Actuall Asserting of Writes");
    ReadWordTest(waddrA, waddrB, wdataA, wdataB);
    endOutput(1);
    endOutput(0);
endtask

endmodule