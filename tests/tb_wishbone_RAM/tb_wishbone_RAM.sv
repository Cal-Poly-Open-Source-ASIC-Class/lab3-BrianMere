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
    $dumpfile("tb_wishbone_RAM.vcd");
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

logic[10:0] addrA, addrB;
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
        $display("Write: [%h] => A@0x%h, [%h] => B@0x%h", dataA[7:0], addrA, dataB[7:0], addrB);
        WriteTest(addrA, addrB, dataA[7:0], dataB[7:0], 0);

        // A alone, B alone (words)
        $display("Words: [%h] => A@0x%h, [%h] => B@0x%h", dataA, addrA & ~11'b11, dataB, addrB & ~11'b11);
        WriteWordTest(addrA & ~11'b11, addrB & ~11'b11, dataA, dataB);

        // A, B together (stall)
        generateValues();
        addrA = addrB; addrA[4] = ~addrA[4];
        $display("Stall: [%h] => A@0x%h, [%h] => B@0x%h", dataA[7:0], addrA, dataB[7:0], addrB);
        WriteTest(addrA, addrB, dataA[7:0], dataB[7:0], 1);
        generateValues();
        addrA = addrB; addrA[4] = ~addrA[4];
        $display("Stall w/ Words: [%h] => A@0x%h, [%h] => B@0x%h", dataA, addrA & ~11'b11, dataB, addrB & ~11'b11);
        WriteWordTest(addrA & ~11'b11, addrB & ~11'b11, dataA, dataB);
    end

    $finish;
end

/** generate values for the addresses and data's */
task generateValues();
    addrA = random()[10:0];
    addrB = random()[10:0];
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
task readInput(input logic[10:0] raddr, input logic use_A = 1);
    if(use_A) begin 
        pA_wb_addr_i = raddr;
        pA_wb_data_i = 32'hDEADBEEF; // this shouldn't ever be used!
        pA_wb_we_i   = 0;
        pA_wb_sel_i  = 1 << raddr[1:0];
        pA_wb_stb_i  = 1;
        pA_wb_cyc_i  = 1;
    end else begin 
        pB_wb_addr_i = raddr;
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

/** End any asserts/reads of outputs with this line */
task endWriteInput(input logic use_A = 1);
    endInput(use_A);
    if(use_A) pA_wb_we_i = 0; else pB_wb_we_i = 0;
endtask

/** Try to write data at `waddr` with the corresponding `wdata`. 
    Assign via port A if `use_A` is asserted. Else, use B. 

    NO CLOCKING
*/
task writeInput(input logic[10:0] waddr, input logic[7:0] wdata, input logic use_A = 1);
    if(use_A) begin 
        pA_wb_addr_i = waddr;
        pA_wb_data_i = {24'b0, wdata} << (8 * waddr[1:0] );
        pA_wb_we_i   = 1;
        pA_wb_sel_i  = 1 << waddr[1:0];
        pA_wb_stb_i  = 1;
        pA_wb_cyc_i  = 1;
    end else begin 
        pB_wb_addr_i = waddr;
        pB_wb_data_i = {24'b0, wdata} << (8 * waddr[1:0] );
        pB_wb_we_i   = 1;
        pB_wb_sel_i  = 1 << waddr[1:0];
        pB_wb_stb_i  = 1;
        pB_wb_cyc_i  = 1;
    end
endtask

/** Try to read data at `raddr` */
task readInputWord(input logic[10:0] raddr, input logic use_A = 1);
    readInput(raddr, use_A);
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
task automatic ReadTest(input logic[10:0] raddrA, input logic[10:0] raddrB, input logic[7:0] rdataA, input logic[7:0] rdataB, input logic clock = 1,
    input logic doA = 1, input logic doB = 1);

    endInput(0);
    endInput(1);

    if(clock) begin 
        if(doA) readInput(raddrA, 1);
        if(doB) readInput(raddrB, 0);
        #TIME_PERIOD;
        if(doA) endInput(1);
        if(doA) endInput(0);
        if (doA && doB) begin 
            assert(
                (({24'b0, rdataA} << (8 * raddrA[1:0] )) == (pA_wb_data_o & (32'hFF << (8 * raddrA[1:0])))) ||
                (({24'b0, rdataB} << (8 * raddrB[1:0] )) == (pB_wb_data_o & (32'hFF << (8 * raddrB[1:0]))))
            ) else $error(
                "Stall: Combined R/W gave an output of neither inputs. See previous displays, but we got outputs A=>0x%h exp: %h and B=>0x%h exp: %h", 
                (pA_wb_data_o & (32'hFF << (8 * raddrA[1:0]))), ({24'b0, rdataA} << (8 * raddrA[1:0] )),
                (pB_wb_data_o & (32'hFF << (8 * raddrB[1:0]))), ({24'b0, rdataB} << (8 * raddrB[1:0] )), 
            );
        end
        else begin 
            if(doA) assert((({24'b0, rdataA} << (8 * raddrA[1:0] )) == (pA_wb_data_o & (32'hFF << (8 * raddrA[1:0]))))) 
                else $error("A: Read data was expected %h but got %h", 
                ({24'b0, rdataA} << (8 * raddrA[1:0] )), (pA_wb_data_o & (32'hFF << (8 * raddrA[1:0]))), ({24'b0, rdataA} << (8 * raddrA[1:0] )));
            if(doB) assert((({24'b0, rdataB} << (8 * raddrB[1:0] )) == (pB_wb_data_o & (32'hFF << (8 * raddrB[1:0]))))) 
                else $error("B: Read data was expected %h but got %h", 
                ({24'b0, rdataB} << (8 * raddrB[1:0] )), (pB_wb_data_o & (32'hFF << (8 * raddrB[1:0]))), ({24'b0, rdataB} << (8 * raddrB[1:0] )));
        end
        if(doA) endOutput(1);
        if(doB) endOutput(0);
    end else begin 
        if(doA) begin
            // A case:
            readInput(raddrA, 1);
            #TIME_PERIOD;
            endInput(1);
            assert(({24'b0, rdataA} << (8 * raddrA[1:0] )) == (pA_wb_data_o & (32'hFF << (8 * raddrA[1:0])))) else $error("A: Read data was expected %h but got %h", 
                ({24'b0, rdataA} << (8 * raddrA[1:0] )), (pA_wb_data_o & (32'hFF << (8 * raddrA[1:0]))));
            endOutput(1);
        end

        if(doB) begin
            // B case:
            readInput(raddrB, 0);
            #TIME_PERIOD;
            endInput(0);
            assert(({24'b0, rdataB} << (8 * raddrB[1:0] )) == (pB_wb_data_o & (32'hFF << (8 * raddrB[1:0])))) else $error("B: Read data was expected %h but got %h", 
                ({24'b0, rdataB} << (8 * raddrB[1:0] )), (pB_wb_data_o & (32'hFF << (8 * raddrB[1:0]))));
            endOutput(0);
        end
    end
endtask

/** Tests writes on separate ports (clocking in between with `clock=0` or not `clock=1`), asserting with `rdataA`, ... */
task automatic WriteTest(input logic[10:0] waddrA, input logic[10:0] waddrB, input logic[7:0] wdataA, input logic[7:0] wdataB, input logic clock = 1);

    endWriteInput(0);
    endWriteInput(1);

    if(clock) begin 
        writeInput(waddrA, wdataA, 1);
        writeInput(waddrB, wdataB, 0);
        #TIME_PERIOD;
        #TIME_PERIOD;
        endInput(1);
        endInput(0);
        $display("Write test BOTH:::");
        ReadTest(waddrA, waddrB, wdataA, wdataB, clock, 1, 1);
    end else begin 
        // A case:
        writeInput(waddrA, wdataA, 1);
        #TIME_PERIOD;
        endWriteInput(1);
        $display("Write test A:::");
        ReadTest(waddrA, waddrB, wdataA, wdataB, clock, 1, 0);
        
        // B case:
        writeInput(waddrB, wdataB, 0);
        #TIME_PERIOD;
        endWriteInput(0);
        $display("Write test B:::");
        ReadTest(waddrA, waddrB, wdataA, wdataB, clock, 0, 1);
    end
    endOutput(1);
    endOutput(0);
endtask

/** Tests reads on separate ports (see soleReadTest()) but now by word, which must clock together here (assume you did more granular tests like Read/WriteTest) */
task automatic ReadWordTest(input logic[10:0] raddrA, input logic[10:0] raddrB, input logic [31:0] rdataA, input logic[31:0] rdataB, input logic clock = 1,
    input logic doA = 1, input logic doB = 1);
    endInput(0);
    endInput(1);

    if(clock) begin 
        if(doA) readInput(raddrA, 1);
        if(doB) readInput(raddrB, 0);
        #TIME_PERIOD;
        if(doA) endInput(1);
        if(doA) endInput(0);
        if(doA && doB) begin 
            assert(pA_wb_data_o == rdataA || pB_wb_data_o == rdataB) 
            else $error(
                "Stall (Word): Combined R/W gave an output of neither inputs. See previous displays, but we got outputs A=>0x%h exp: %h and B=>0x%h exp: %h", 
                pA_wb_data_o, rdataA,
                pB_wb_data_o, rdataB
            );
        end else begin
            if(doA) assert(pA_wb_data_o == rdataA) else $error("A: Tried reading word got %h expected %h", pA_wb_data_o, rdataA);
            if(doB) assert(pB_wb_data_o == rdataB) else $error("B: Tried reading word got %h expected %h", pB_wb_data_o, rdataB);
        end
        if(doA) endOutput(1);
        if(doB) endOutput(0);
    end else begin 
        if(doA) begin
            // A case:
            readInput(raddrA, 1);
            #TIME_PERIOD;
            endInput(1);
            assert(pA_wb_data_o == rdataA) else $error("A: Tried reading word got %h expected %h", pA_wb_data_o, rdataA);
            endOutput(1);
        end

        if(doB) begin
            // B case:
            readInput(raddrB, 0);
            #TIME_PERIOD;
            endInput(0);
            assert(pB_wb_data_o == rdataB) else $error("B: Tried reading word got %h expected %h", pB_wb_data_o, rdataB);
            endOutput(0);
        end
    end    
endtask

/** Tests writes on separate ports (see soleReadTest()) but now by word, which must clock together here (assume you did more granular tests like Read/WriteTest) */
task automatic WriteWordTest(input logic[10:0] waddrA, input logic[10:0] waddrB, input logic [31:0] wdataA, input logic[31:0] wdataB, input logic clock = 1, 
    input logic doA = 1, input logic doB = 1);
    endWriteInput(0);
    endWriteInput(1);

    if(clock) begin 
        writeInputWord(waddrA, wdataA, 1);
        writeInputWord(waddrB, wdataB, 0);
        #TIME_PERIOD;
        #TIME_PERIOD;
        endInput(1);
        endInput(0);
        $display("Write test BOTH:::");
        ReadWordTest(waddrA, waddrB, wdataA, wdataB, clock, 1, 1);
    end else begin 
        // A case:
        writeInputWord(waddrA, wdataA, 1);
        #TIME_PERIOD;
        endWriteInput(1);
        $display("Write test A:::");
        ReadWordTest(waddrA, waddrB, wdataA, wdataB, clock, 1, 0);
        
        // B case:
        writeInputWord(waddrB, wdataB, 0);
        #TIME_PERIOD;
        endWriteInput(0);
        $display("Write test B:::");
        ReadWordTest(waddrA, waddrB, wdataA, wdataB, clock, 0, 1);
    end
    endOutput(1);
    endOutput(0);
endtask

endmodule