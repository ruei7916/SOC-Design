module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    // AXI-Lite AW channel
    output  wire                     awready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    // AXI-Lite W channel
    output  wire                     wready,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    // AXI-Lite AR channel
    output  reg                      arready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    // AXI-Lite R channel
    input   wire                     rready,
    output  reg                      rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,
    // AXI-Stream slave
    output  wire                     ss_tready,
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    // AXI-Stream master
    input   wire                     sm_tready, 
    output  reg                      sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    // bram for tap RAM
    output  reg  [3:0]               tap_WE,
    output  reg                      tap_EN,
    output  reg  [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,
    // bram for data RAM
    output  reg  [3:0]               data_WE,
    output  reg                      data_EN,
    output  reg  [(pDATA_WIDTH-1):0] data_Di,
    output  reg  [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);


// write transaction control signal: wready, awready
// wait for both addr and data then receive them at the same time
assign wready = wvalid & awvalid;
assign awready = wvalid & awvalid;
// read transaction control signal: arready
always @(*) begin
    // check if receiveing write request or rdata is received by master
    if((~wready)&(~awready)&(~rvalid))begin 
        arready = arvalid;
    end
    else begin
        arready = 1'b0;
    end
end
reg _rvalid, rvalid_d;
reg [1:0] _rdata_muxsel1, rdata_muxsel1;

reg [(pDATA_WIDTH-1):0] data_length, _data_length;
reg [(pDATA_WIDTH-1):0] rdata_reg, _rdata_reg;
reg [(pADDR_WIDTH-1):0] config_tap_A;
reg [(pDATA_WIDTH-1):0] data_idx, _data_idx;

reg ap_start, _ap_start;
reg ap_idle, _ap_idle;
reg [1:0] ap_done, _ap_done;


assign write_req_start = wready & awready & wdata[0] & (awaddr==12'h00);

// rdata
assign rdata = _rdata_reg;
always @(*) begin
    case (rdata_muxsel1)
        2'b00: _rdata_reg = {5'b0, ap_idle, ap_done[0], ap_start};
        2'b01: _rdata_reg = tap_Do;
        2'b10: _rdata_reg = data_length;
        2'b11: _rdata_reg = rdata_reg;
    endcase
end
// processing axi-lite write/read request
always @(*) begin
    _ap_start = 1'b0; // ap_start only stays high for 1 cycle
    _data_length = data_length;
    config_tap_A = 0; // 
    tap_WE = 4'b0000;
    tap_EN = 1'b1;
    tap_Di = wdata;
    _rvalid = rvalid;
    _rdata_muxsel1 = 2'b11;
    if(wready&awready)begin // write request
        if(write_req_start)begin
            _ap_start = 1'b1;
        end
        else if(awaddr==12'h10)begin
            _data_length = wdata;
        end
        else if(awaddr[7])begin // 0x80: writing tap param
            config_tap_A = awaddr & 12'h3f;
            tap_WE = 4'b1111;
        end
    end
    else if(arready&arvalid)begin // read request
        _rvalid = 1'b1;
        if(araddr==12'h00)begin
            _rdata_muxsel1 = 2'b00;
        end
        else if(araddr==12'h10)begin // data length
            _rdata_muxsel1 = 2'b10;
        end
        else if(araddr[7])begin // 0x80: reading tap param
            _rdata_muxsel1 = 2'b01;
            config_tap_A = araddr & 12'h3f;
        end
    end
    else begin
        if(rvalid&rready) _rvalid = 1'b0; // stop asserting rvalid
    end
end
// ap_idle
always @(*) begin
    _ap_idle = ap_idle;
    if(write_req_start)begin // start fir
        _ap_idle = 1'b0;
    end
    else if(data_idx==data_length)begin
        _ap_idle = 1'b1;
    end
end
// ap_done
// ap_done[1]: fir started
// ap_done[0]: fir finished transmittion
always @(*) begin
    _ap_done = ap_done;
    if(write_req_start)begin // start fir
        _ap_done = 2'b10;
    end
    else if(arready & arvalid && (ap_done==2'b11) && (araddr==12'h00))begin // read request
        _ap_done = 2'b00; // reset ap_done
    end
    else if((sm_tvalid==1'b0) && (data_idx==data_length) && ap_done[1] )begin // finish transmition
        _ap_done = 2'b11;
    end
end

// reg
always @(posedge axis_clk) begin
    if(~axis_rst_n)begin
        rvalid <= 1'b0;
        rvalid_d <= 1'b0;
        ap_start <= 1'b0;
        ap_idle  <= 1'b1;
        ap_done  <= 2'b00;
        data_length <= 0;
        rdata_reg <= 0;
        rdata_muxsel1 <= 1'b0;
    end
    else begin
        rvalid <= _rvalid;
        rvalid_d <= rvalid;
        ap_start <= _ap_start;
        ap_idle  <= _ap_idle;
        ap_done  <= _ap_done;
        data_length <= _data_length;
        rdata_reg <= _rdata_reg;
        rdata_muxsel1 <= _rdata_muxsel1;
    end
end

reg [3:0] tap_idx, _tap_idx;
reg mul_data_in_sel, _mul_data_in_sel;
// fir engine
reg [(pDATA_WIDTH-1):0] acc, _acc, mul_out;
reg stall;
wire acc_reset;
assign sm_tdata = _acc;
// avoid reading x from sram
always @(*) begin
    if( tap_idx > data_idx)begin
        _mul_data_in_sel = 1'b1;
    end
    else begin
        _mul_data_in_sel = 1'b0;
    end
end
always @(posedge axis_clk) begin
    mul_data_in_sel <= _mul_data_in_sel;
end
always @(*) begin
    mul_out = (mul_data_in_sel ? 0 : data_Do) * tap_Do;
    _acc = (acc_reset ? 0 : acc) + (stall ? 0 : mul_out);
end
always @(posedge axis_clk) begin
    if(~axis_rst_n)begin
        acc <= 0;
    end
    else begin
        acc <= _acc;
    end
end

// sm stall
always @(*) begin
    if(({sm_tvalid,sm_tready}=={1'b1,1'b0}||ss_tvalid==0))
        stall = 1'b1;
    else 
        stall = 1'b0;
end
assign sm_tlast = (data_idx==data_length) ? sm_tvalid : 1'b0;

reg [3:0] data_A_shift, _data_A_shift;
reg _sm_tvalid;
assign acc_reset = (tap_idx==4'd9) ? 1'b1 : 1'b0;
assign ss_tready = (tap_idx==4'd0) ? 1'b1 : 1'b0;

always @(*) begin
    _tap_idx = tap_idx;
    _data_idx = data_idx;
    _data_A_shift = data_A_shift;
    if(~ap_idle)begin // check fir started
        case (tap_idx)
            4'd0: begin
                _tap_idx = 4'd10;
                _data_idx = data_idx + 1;
                _data_A_shift = (data_A_shift==4'd10) ? 4'd0 : data_A_shift + 1;
            end
            4'd10: begin
                if(stall) _tap_idx = 4'd10;
                else _tap_idx = 4'd9;
            end
            default: begin
                _tap_idx = tap_idx - 4'd1;
            end
        endcase
    end
    else begin
        if(_ap_start==1'b1)begin
            _tap_idx = 4'd10;
            _data_idx = 0;
            _data_A_shift = 4'd0;
        end 
    end
end
// sm_tvalid
always @(*) begin
    _sm_tvalid = sm_tvalid;
    if((tap_idx==4'd0) && (ap_idle==1'b0))
        _sm_tvalid = 1'b1;
    else if(sm_tready & sm_tvalid)
        _sm_tvalid = 1'b0;
end
always @(posedge axis_clk) begin
    if(~axis_rst_n)begin
        tap_idx <= 4'd10;
        data_idx <= 0;
        sm_tvalid <= 1'b0;
        data_A_shift <= 4'd0;
    end
    else begin
        tap_idx <= _tap_idx;
        data_idx <= _data_idx;
        sm_tvalid <= _sm_tvalid;
        data_A_shift <= _data_A_shift;
    end
end

// ss
wire [(pADDR_WIDTH-1):0] fir_tap_A;
assign tap_A = ap_idle ? config_tap_A : fir_tap_A;
assign fir_tap_A = tap_idx << 2;

always @(*) begin
    data_Di = 0;
    data_WE = 4'b0000;
    data_EN = 1'b1;
    if(tap_idx==4'd0)begin //read in data
        data_Di = ss_tdata;
        data_WE = 4'b1111;
    end
end
reg signed [5:0] data_ram_idx;
reg [3:0] idx_t;
always @(*) begin
    data_ram_idx = tap_idx - data_A_shift;
    idx_t = data_ram_idx + 4'd11;
    if(data_ram_idx>=0)
        data_A = data_ram_idx << 2;
    else 
        data_A = idx_t << 2;
end


endmodule