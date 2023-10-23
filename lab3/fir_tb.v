`define CYCLE 14

module fir_tb
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11,
    parameter Data_Num    = 600
)();

// ===== module I/O ===== //
    wire                        awready;
    wire                        wready;
    reg                         awvalid;
    reg   [(pADDR_WIDTH-1): 0]  awaddr;
    reg                         wvalid;
    reg signed [(pDATA_WIDTH-1) : 0] wdata;
    wire                        arready;
    reg                         rready;
    reg                         arvalid;
    reg         [(pADDR_WIDTH-1): 0] araddr;
    wire                        rvalid;
    wire signed [(pDATA_WIDTH-1): 0] rdata;
    reg                         ss_tvalid;
    reg signed [(pDATA_WIDTH-1) : 0] ss_tdata;
    reg                         ss_tlast;
    wire                        ss_tready;
    reg                         sm_tready;
    wire                        sm_tvalid;
    wire signed [(pDATA_WIDTH-1) : 0] sm_tdata;
    wire                        sm_tlast;
    reg                         axis_clk;
    reg                         axis_rst_n;

    // ram for tap
    wire [3:0]               tap_WE;
    wire                     tap_EN;
    wire [(pDATA_WIDTH-1):0] tap_Di;
    wire [(pADDR_WIDTH-1):0] tap_A;
    wire [(pDATA_WIDTH-1):0] tap_Do;

    // ram for data RAM
    wire [3:0]               data_WE;
    wire                     data_EN;
    wire [(pDATA_WIDTH-1):0] data_Di;
    wire [(pADDR_WIDTH-1):0] data_A;
    wire [(pDATA_WIDTH-1):0] data_Do;

// ===== instantiation ===== //
    fir fir_DUT(
        .awready(awready),
        .wready(wready),
        .awvalid(awvalid),
        .awaddr(awaddr),
        .wvalid(wvalid),
        .wdata(wdata),
        .arready(arready),
        .rready(rready),
        .arvalid(arvalid),
        .araddr(araddr),
        .rvalid(rvalid),
        .rdata(rdata),
        .ss_tvalid(ss_tvalid),
        .ss_tdata(ss_tdata),
        .ss_tlast(ss_tlast),
        .ss_tready(ss_tready),
        .sm_tready(sm_tready),
        .sm_tvalid(sm_tvalid),
        .sm_tdata(sm_tdata),
        .sm_tlast(sm_tlast),

        // ram for tap
        .tap_WE(tap_WE),
        .tap_EN(tap_EN),
        .tap_Di(tap_Di),
        .tap_A(tap_A),
        .tap_Do(tap_Do),

        // ram for data
        .data_WE(data_WE),
        .data_EN(data_EN),
        .data_Di(data_Di),
        .data_A(data_A),
        .data_Do(data_Do),

        .axis_clk(axis_clk),
        .axis_rst_n(axis_rst_n)

        );
    
    // RAM for tap
    bram11 tap_RAM (
        .CLK(axis_clk),
        .WE(tap_WE),
        .EN(tap_EN),
        .Di(tap_Di),
        .A(tap_A),
        .Do(tap_Do)
    );

    // RAM for data: choose bram11 or bram12
    bram11 data_RAM(
        .CLK(axis_clk),
        .WE(data_WE),
        .EN(data_EN),
        .Di(data_Di),
        .A(data_A),
        .Do(data_Do)
    );


// ===== dump waveform ===== //
initial begin
    $dumpfile("fir.vcd");
    $dumpvars();
end

// ===== system clock ===== //
initial begin
    axis_clk = 0;
    forever begin
        #(`CYCLE/2) axis_clk = (~axis_clk);
    end
end

// ===== cycle accumulator ===== //
integer total_cycle;
initial begin
    total_cycle = 0;
    while(1) begin
        @(posedge axis_clk);
        total_cycle = total_cycle + 1;
    end
end

// ===== Prevent simulation hang ===== //
integer timeout = (1000000);
initial begin
    while(timeout > 0) begin
        @(posedge axis_clk);
        timeout = timeout - 1;
    end
    $display($time, "Simualtion Hang ....");
    $finish;
end

//--------------------------main----------------------------//
reg signed [31:0] coef[0:10];
reg signed [(pDATA_WIDTH-1):0] Din_list[0:(Data_Num-1)];
reg signed [(pDATA_WIDTH-1):0] golden_list[0:(Data_Num-1)];

reg [31:0]  data_length;
integer Din, golden, input_data, golden_data, m;
integer i, j, k;

reg error_coef, error;
reg pass2, pass3;
reg start_fir;

reg ap_done;

initial begin
// coef config
    coef[0]  =  32'd0;
    coef[1]  = -32'd10;
    coef[2]  = -32'd9;
    coef[3]  =  32'd23;
    coef[4]  =  32'd56;
    coef[5]  =  32'd63;
    coef[6]  =  32'd56;
    coef[7]  =  32'd23;
    coef[8]  = -32'd9;
    coef[9]  = -32'd10;
    coef[10] =  32'd0;
// read input and golden from files
    data_length = 0;
    Din = $fopen("./samples_triangular_wave.dat","r");
    golden = $fopen("./out_gold.dat","r");
    for(m=0;m<Data_Num;m=m+1) begin
        input_data = $fscanf(Din,"%d", Din_list[m]);
        golden_data = $fscanf(golden,"%d", golden_list[m]);
        data_length = data_length + 1;
    end
    $fclose(Din);
    $fclose(golden);
// init variables
    pass2 = 0; pass3 = 0;
    error_coef = 0; error = 0;
    awvalid = 0; wvalid = 0; arvalid = 0;
    ss_tvalid = 0;
    ap_done = 0;
    start_fir = 0;
// reset
    axis_rst_n = 0;
    @(posedge axis_clk);
    @(posedge axis_clk);
    @(negedge axis_clk);
    axis_rst_n = 1;
// coef setup
    awvalid <= 0; wvalid <= 0; arvalid <= 0;
    @(posedge axis_clk);
    $display("==================== setup phase ====================");
    $display("----Start the coefficient input(AXI-lite)----");
    config_write(12'h10, data_length); // write data length
    for(j=0; j < Tape_Num; j=j+1) begin
        config_write(12'h80+4*j, coef[j]); // write coef
    end
    awvalid <= 0; wvalid <= 0;
    // read-back and check
    @(posedge axis_clk);
    $display(" Check Coefficient ...");
    for(j=0; j < Tape_Num; j=j+1) begin
        config_read_check(12'h80+4*j, coef[j], 32'hffffffff);
    end
    $display(" Tap programming done ...");
    $display("----End the coefficient input(AXI-lite)----");
    $display("*** Start FIR");
    @(posedge axis_clk) config_write(12'h00, 32'h0000_0001);    // ap_start = 1
    awvalid <= 0; wvalid <= 0; start_fir <= 1;
// input feeding start
    ss_tvalid = 0;
    // pass1
    $display("------------Start simulation pass1-----------");
    $display("----Start the data input(AXI-Stream)----");
    for(i=0;i<(data_length-1);i=i+1) begin
        ss_tlast = 0; ss(Din_list[i]);
    end
    ss_tvalid = 0;
    config_read_check(12'h00, 32'h00, 32'h0000_000f); // check idle = 0
    ss_tlast = 1; ss(Din_list[(Data_Num-1)]);
    ss_tvalid = 0;
    $display("------End the data input(AXI-Stream)------");
    // pass2
    wait(pass2);
    @(posedge axis_clk);
    config_write(12'h00, 32'h0000_0001);    // ap_start = 1
    awvalid <= 0; wvalid <= 0;
    ss_tvalid = 0;
    $display("------------Start simulation pass2-----------");
    $display("----Start the data input(AXI-Stream)----");
    for(i=0;i<(data_length-1);i=i+1) begin
        ss_tlast = 0; ss(Din_list[i]);
    end
    ss_tvalid = 0;
    config_read_check(12'h00, 32'h00, 32'h0000_000f); // check idle = 0
    ss_tlast = 1; ss(Din_list[(Data_Num-1)]);
    ss_tvalid = 0;
    $display("------End the data input(AXI-Stream)------");
    // pass3
    wait(pass3);
    @(posedge axis_clk);
    config_write(12'h00, 32'h0000_0001);    // ap_start = 1
    awvalid <= 0; wvalid <= 0;
    ss_tvalid = 0;
    $display("------------Start simulation pass3-----------");
    $display("----Start the data input(AXI-Stream)----");
    for(i=0;i<(data_length-1);i=i+1) begin
        ss_tlast = 0; ss(Din_list[i]);
    end
    ss_tvalid = 0;
    config_read_check(12'h00, 32'h00, 32'h0000_000f); // check idle = 0
    ss_tlast = 1; ss(Din_list[(Data_Num-1)]);
    ss_tvalid = 0;
    $display("------End the data input(AXI-Stream)------");
end


// ==== output data compare ==== //
reg status_error;
initial begin
    status_error = 0;
    wait(axis_rst_n);
    sm_tready = 0;
    @(posedge axis_clk)
    for(k=0;k < data_length;k=k+1) begin
        sm(golden_list[k],k);
    end
    //config_read_check(12'h00, 32'h02, 32'h0000_0002); // check ap_done = 1 (0x00 [bit 1])
    //config_read_check(12'h00, 32'h04, 32'h0000_0004); // check ap_idle = 1 (0x00 [bit 2])
    wait(ap_done);
    if (error == 0 & error_coef == 0) begin
        $display("\033[1;32m------------------------------------------\033[0m");
        $display("\033[1;32m-------------Congratulations!-------------\033[0m");
        $display("\033[1;32m----------------Simulation1---------------\033[0m");
        $display("\033[1;32m----------- ^_^    PASS    ^_^ -----------\033[0m");
        $display("\033[1;32m------------------------------------------\033[0m");
    end
    else begin
        $display("\033[1;31m----------Simulation1 Failed--------------\033[0m");
    end
    $display("Total cycle count = %0d", total_cycle);
    // pass2
    @(posedge axis_clk)
    pass2 <= 1; total_cycle <= 0;
    error = 0; status_error = 0;
    sm_tready = 0;
    ap_done = 0;
    @(posedge axis_clk)
    for(k=0;k < data_length;k=k+1) begin
        sm(golden_list[k],k);
    end
    //config_read_check(12'h00, 32'h02, 32'h0000_0002); // check ap_done = 1 (0x00 [bit 1])
    //config_read_check(12'h00, 32'h04, 32'h0000_0004); // check ap_idle = 1 (0x00 [bit 2])
    wait(ap_done);
    if (error == 0 & error_coef == 0) begin
        $display("\033[1;32m------------------------------------------\033[0m");
        $display("\033[1;32m-------------Congratulations!-------------\033[0m");
        $display("\033[1;32m----------------Simulation2---------------\033[0m");
        $display("\033[1;32m----------- ^_^    PASS    ^_^ -----------\033[0m");
        $display("\033[1;32m------------------------------------------\033[0m");
    end
    else begin
        $display("\033[1;31m----------Simulation2 Failed--------------\033[0m");
    end
    $display("Total cycle count = %0d", total_cycle);
    // pass3
    @(posedge axis_clk)
    pass3 <= 1; total_cycle <= 0;
    error = 0; status_error = 0;
    sm_tready = 0;
    ap_done = 0;
    @(posedge axis_clk)
    for(k=0;k < data_length;k=k+1) begin
        sm(golden_list[k],k);
    end
    //config_read_check(12'h00, 32'h02, 32'h0000_0002); // check ap_done = 1 (0x00 [bit 1])
    //config_read_check(12'h00, 32'h04, 32'h0000_0004); // check ap_idle = 1 (0x00 [bit 2])
    wait(ap_done);
    if (error == 0 & error_coef == 0) begin
        $display("\033[1;32m------------------------------------------\033[0m");
        $display("\033[1;32m-------------Congratulations!-------------\033[0m");
        $display("\033[1;32m----------------Simulation3---------------\033[0m");
        $display("\033[1;32m----------- ^_^    PASS    ^_^ -----------\033[0m");
        $display("\033[1;32m------------------------------------------\033[0m");
    end
    else begin
        $display("\033[1;31m----------Simulation3 Failed--------------\033[0m");
    end
    $display("Total cycle count = %0d", total_cycle);
    $finish;
end


// ==== polling ap_done ==== //
initial begin
    wait(start_fir);
    while(1) check_ap_done;
end


    task check_ap_done;
        begin
            arvalid <= 1; araddr <= 12'h00;
            rready <= 1;
            @(posedge axis_clk);
            while (!arready) @(posedge axis_clk);
            arvalid <= 0;
            while (!rvalid) @(posedge axis_clk);
            if(rdata&32'h0000_0002)begin //ap_done
                $display("ap_done sampled");
                if(k!==data_length)begin
                    $display("\033[1;31mAn error occured: ap_done sampled but testbench haven't reveived all output data");
                    $display("terminating...");
                    $finish;
                end
                config_read_check(12'h00, 32'h04, 32'h0000_0004); // check ap_idle = 1 (0x00 [bit 2])
                ap_done <= 1;
            end
            else begin
                ap_done <= 0;
            end
        end
    endtask

    task config_write;
        input [11:0]    addr;
        input [31:0]    data;
        begin
            //@(posedge axis_clk);
            awvalid <= 1; awaddr <= addr;
            wvalid  <= 1; wdata <= data;
            @(posedge axis_clk);
            while (!wready) @(posedge axis_clk);
        end
    endtask

    task config_read_check;
        input [11:0]        addr;
        input signed [31:0] exp_data;
        input [31:0]        mask;
        begin
            arvalid <= 1; araddr <= addr;
            rready <= 1;
            @(posedge axis_clk);
            while (!arready) @(posedge axis_clk);
            arvalid <= 0;
            while (!rvalid) @(posedge axis_clk);
            if( (rdata & mask) !== (exp_data & mask)) begin
                $display("ERROR: exp = %d, rdata = %d", exp_data, rdata);
                error_coef <= 1;
            end else begin
                $display("OK: exp = %d, rdata = %d", exp_data, rdata);
            end
        end
    endtask

    task ss;
        input  signed [31:0] in1;
        begin
            ss_tvalid <= 1;
            ss_tdata  <= in1;
            //wait(ss_tready);
            @(posedge axis_clk);
            while (!ss_tready) begin
                @(posedge axis_clk);
            end
        end
    endtask

    task sm;
        input  signed [31:0] in2; // golden data
        input         [31:0] pcnt; // pattern count
        begin
            sm_tready <= 1;
            @(posedge axis_clk) 
            //wait(sm_tvalid);
            while(!sm_tvalid) @(posedge axis_clk);
            if (sm_tdata !== in2) begin
                $display("[ERROR] [Pattern %d] Golden answer: %d, Your answer: %d", pcnt, in2, sm_tdata);
                error <= 1;
            end
            else begin
                $display("[PASS] [Pattern %d] Golden answer: %d, Your answer: %d", pcnt, in2, sm_tdata);
            end
            //@(posedge axis_clk);
        end
    endtask
endmodule

