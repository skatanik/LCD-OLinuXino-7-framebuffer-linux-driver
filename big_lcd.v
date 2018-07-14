module big_lcd(

    input                       clk,
    input                       reset_n,
    input   [23:0]          lcd_readdata,
    output                    lcd_read,

    output                     data_request,
    input                   no_data_available,

    output  [7:0]           R,
    output  [7:0]           G,
    output  [7:0]           B,
    output                     HSYNC,
    output                     VSYNC,
    output                     LCD_CLK

);

reg [10:0] counter_hs;
reg [10:0] counter_vs;

assign LCD_CLK = reset_n & clk;

always@(posedge clk or negedge reset_n)
begin
if(!reset_n)
    begin
        counter_hs <= 11'd0;
        counter_vs <= 11'd0;
    end
    else
        begin
            if(counter_hs == 11'd1055)
            begin

                if(counter_vs == 11'd524)
                    counter_vs <= 11'd0;
                else
                    counter_vs <= counter_vs + 11'd1;

                counter_hs <= 11'd0;
            end
            else
                counter_hs <= counter_hs + 11'd1;
        end
end

assign lcd_read = (counter_hs >= 11'd44 && counter_hs < 11'd845 && counter_vs >= 11'd23  && counter_vs < 11'd503) ;

assign VSYNC = (counter_vs >= 11'd0 && counter_vs < 11'd10) ? 1'b0 : 1'b1;
assign HSYNC = (counter_hs >= 11'd0 && counter_hs < 11'd10) ? 1'b0 : 1'b1;

assign R = !no_data_available ? lcd_readdata[23:16] : 8'h34;
assign G = !no_data_available ? lcd_readdata[15:8] : 8'h34;
assign B = !no_data_available ? lcd_readdata[7:0] : 8'h34;

assign data_request = (counter_hs == 810 && counter_vs == 11'd504);

endmodule
