module lcd_with_dma(

    input               clk_lcd,
    input               reset_lcd,

    input               clk_dma,
    input               reset_dma,

    // mm-avalon ctrl
    input [1:0]         reg_avl_addr,
    input [31:0]        reg_avl_writedata,
    input               reg_avl_write,


    // mm-avalon DMA
    output [29:0]       dma_avl_addr,
    output              dma_avl_read,
    output  [7:0]       dma_avl_burstcount,

    input [31:0]        dma_avl_readdata,
    input               dma_avl_readdatavalid,
    input               dma_avl_waitrequest,

    // conduit
    output  [7:0]       R,
    output  [7:0]       G,
    output  [7:0]       B,
    output              HSYNC,
    output              VSYNC,
    output              LCD_CLK
);

localparam [7:0]            MAX_TRANSFER_SIZE   = 8'd32;  // in words
localparam [23:0]           FRAME_SIZE          = 480*800;
wire [31:0] key;
assign key = 32'h96a6a3cd;
/******************* Control And Status Registers  ***********************/

localparam                  CTRL_ENABLE_BIT     = 0;

reg [31:0] reg_start_address;
reg [31:0] reg_cntrl;
reg [31:0] reg_key;


always @(posedge clk_dma or negedge reset_dma)
    if(!reset_dma)		                                    reg_start_address <= 32'd0;
    else if(reg_avl_write && !reg_cntrl[CTRL_ENABLE_BIT] && (reg_avl_addr == 2'b01))
                                                            reg_start_address <= reg_avl_writedata;

always @(posedge clk_dma or negedge reset_dma)
    if(!reset_dma)                                          reg_key <= 32'd0;
    else if(reg_avl_write && !reg_cntrl[CTRL_ENABLE_BIT] && (reg_avl_addr == 2'b10))
                                                            reg_key <= reg_avl_writedata;

always @(posedge clk_dma or negedge reset_dma)
    if(!reset_dma)                                      reg_cntrl <= 32'd0;
    else if(reg_avl_write && (reg_avl_addr == 2'b00))   reg_cntrl <= reg_avl_writedata;

wire key_valid;
assign key_valid = (reg_key == key);

/***************************  Writing to LCD FIFO  **********************************/

/**** LCD, FIFO ****/


wire data_request;
reg [1:0] data_request_sync;

always @(posedge clk_dma or negedge reset_dma)
    if(!reset_dma)          data_request_sync <= 2'b00;
    else                    data_request_sync <= {data_request_sync[0], data_request};

wire        wrfull_sig;
wire [9:0]  wrusedw_sig;
wire        aclr_sig;

wire [31:0] lcd_readdata;
wire        lcd_read;
wire        no_data_available;

fifo_lcd	fifo_lcd_inst (
	.aclr ( data_request_sync[1] ),
	.data ( dma_avl_readdata ),
    .wrclk ( clk_dma ),
	.wrreq ( dma_avl_readdatavalid ),
    .wrfull ( wrfull_sig ),
	.wrusedw ( wrusedw_sig ),

	.rdclk ( clk_lcd ),
	.rdreq ( lcd_read ),
	.q ( lcd_readdata ),
	.rdempty ( no_data_available )
	);

big_lcd big_lcd_1(
    .clk(clk_lcd),
    .reset_n(reset_lcd),
    .lcd_readdata(lcd_readdata[23:0]),
    .lcd_read(lcd_read),

    .data_request(data_request),
    .no_data_available(no_data_available),

    .R(R),
    .G(G),
    .B(B),
    .HSYNC(HSYNC),
    .VSYNC(VSYNC),
    .LCD_CLK(LCD_CLK)
);


/****  FSM  ****/
reg [29:0] current_address;

localparam [1:0] STATE_IDLE         = 2'b00;
localparam [1:0] STATE_SEND_RQ      = 2'b01;
localparam [1:0] STATE_WAIT         = 2'b10;


reg [1:0] current_state;
reg [1:0] next_state;

reg [9:0] requests_counter;

always @(posedge clk_dma or negedge reset_dma)
    if(!reset_dma)
        current_state <= STATE_IDLE;
     else
        current_state <= next_state;

always @*
    begin
        case(current_state)

            STATE_IDLE:
                next_state = (data_request_sync[1] && reg_cntrl[0] && key_valid) ? STATE_SEND_RQ : STATE_IDLE;

            STATE_SEND_RQ:
                next_state = !dma_avl_waitrequest ? STATE_WAIT : STATE_SEND_RQ;

            STATE_WAIT:
                next_state = ( (reg_start_address + FRAME_SIZE) == current_address) ? STATE_IDLE :
                         ( ((wrusedw_sig < 10'd512) && (requests_counter < 10'd10))  ? STATE_SEND_RQ : STATE_WAIT);

            default:
            next_state = STATE_IDLE;

        endcase
    end

reg [6:0] transfers_counter;

wire start_sending_request;
wire start_waiting;
wire transaction_finished;

reg dma_avl_readdatavalid_delayed;

always @(posedge clk_dma or negedge reset_dma)
    if(!reset_dma)      dma_avl_readdatavalid_delayed <= 1'b0;
    else                dma_avl_readdatavalid_delayed <= dma_avl_readdatavalid;

assign start_sending_request    = (next_state == STATE_SEND_RQ) && (current_state != STATE_SEND_RQ);
assign start_waiting            = (next_state == STATE_WAIT) && (current_state == STATE_SEND_RQ);
assign transaction_finished     = (transfers_counter == MAX_TRANSFER_SIZE);


always @(posedge clk_dma or negedge reset_dma)
    if(!reset_dma)                                  transfers_counter <= 7'd0;
    else if(transaction_finished || !reg_cntrl[0])  transfers_counter <= 7'd0;
    else                                            transfers_counter <= transfers_counter + {6'd0, dma_avl_readdatavalid};

always @(posedge clk_dma or negedge reset_dma)
    if(!reset_dma)          requests_counter <= 10'h0;
    else if(!reg_cntrl[0])  requests_counter <= 10'h0;
    else                    requests_counter <= requests_counter + {9'd0, start_waiting} - {9'd0, transaction_finished};

/**** Address calculating *****/
always @(posedge clk_dma or negedge reset_dma)
    if(!reset_dma)                          current_address <=  30'd0;
    else if(current_state == STATE_IDLE)    current_address <= reg_start_address;
    else if(start_waiting)                  current_address <= current_address + MAX_TRANSFER_SIZE;


/************ SIGNALS FORMING *************/
assign dma_avl_read         = current_state == STATE_SEND_RQ;
assign dma_avl_addr         = dma_avl_read ? current_address : 30'd0;
assign dma_avl_burstcount   = dma_avl_read ? MAX_TRANSFER_SIZE : 8'd0;

endmodule
